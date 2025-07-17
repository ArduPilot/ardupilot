#pragma once
// MESSAGE GNSS_INTEGRITY PACKING

#define MAVLINK_MSG_ID_GNSS_INTEGRITY 441


typedef struct __mavlink_gnss_integrity_t {
 uint32_t system_errors; /*<  Errors in the GPS system.*/
 uint16_t raim_hfom; /*< [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.*/
 uint16_t raim_vfom; /*< [cm] Vertical expected accuracy using satellites successfully validated using RAIM.*/
 uint8_t id; /*<  GNSS receiver id. Must match instance ids of other messages from same receiver.*/
 uint8_t authentication_state; /*<  Signal authentication state of the GPS system.*/
 uint8_t jamming_state; /*<  Signal jamming state of the GPS system.*/
 uint8_t spoofing_state; /*<  Signal spoofing state of the GPS system.*/
 uint8_t raim_state; /*<  The state of the RAIM processing.*/
 uint8_t corrections_quality; /*<  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.*/
 uint8_t system_status_summary; /*<  An abstract value representing the overall status of the receiver, or 255 if not available.*/
 uint8_t gnss_signal_quality; /*<  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.*/
 uint8_t post_processing_quality; /*<  An abstract value representing the estimated PPK quality, or 255 if not available.*/
} mavlink_gnss_integrity_t;

#define MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN 17
#define MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN 17
#define MAVLINK_MSG_ID_441_LEN 17
#define MAVLINK_MSG_ID_441_MIN_LEN 17

#define MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC 169
#define MAVLINK_MSG_ID_441_CRC 169



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GNSS_INTEGRITY { \
    441, \
    "GNSS_INTEGRITY", \
    12, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gnss_integrity_t, id) }, \
         { "system_errors", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gnss_integrity_t, system_errors) }, \
         { "authentication_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gnss_integrity_t, authentication_state) }, \
         { "jamming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gnss_integrity_t, jamming_state) }, \
         { "spoofing_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gnss_integrity_t, spoofing_state) }, \
         { "raim_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gnss_integrity_t, raim_state) }, \
         { "raim_hfom", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_gnss_integrity_t, raim_hfom) }, \
         { "raim_vfom", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_gnss_integrity_t, raim_vfom) }, \
         { "corrections_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gnss_integrity_t, corrections_quality) }, \
         { "system_status_summary", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_gnss_integrity_t, system_status_summary) }, \
         { "gnss_signal_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_gnss_integrity_t, gnss_signal_quality) }, \
         { "post_processing_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gnss_integrity_t, post_processing_quality) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GNSS_INTEGRITY { \
    "GNSS_INTEGRITY", \
    12, \
    {  { "id", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_gnss_integrity_t, id) }, \
         { "system_errors", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_gnss_integrity_t, system_errors) }, \
         { "authentication_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_gnss_integrity_t, authentication_state) }, \
         { "jamming_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_gnss_integrity_t, jamming_state) }, \
         { "spoofing_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_gnss_integrity_t, spoofing_state) }, \
         { "raim_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_gnss_integrity_t, raim_state) }, \
         { "raim_hfom", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_gnss_integrity_t, raim_hfom) }, \
         { "raim_vfom", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_gnss_integrity_t, raim_vfom) }, \
         { "corrections_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_gnss_integrity_t, corrections_quality) }, \
         { "system_status_summary", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_gnss_integrity_t, system_status_summary) }, \
         { "gnss_signal_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_gnss_integrity_t, gnss_signal_quality) }, \
         { "post_processing_quality", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_gnss_integrity_t, post_processing_quality) }, \
         } \
}
#endif

/**
 * @brief Pack a gnss_integrity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  GNSS receiver id. Must match instance ids of other messages from same receiver.
 * @param system_errors  Errors in the GPS system.
 * @param authentication_state  Signal authentication state of the GPS system.
 * @param jamming_state  Signal jamming state of the GPS system.
 * @param spoofing_state  Signal spoofing state of the GPS system.
 * @param raim_state  The state of the RAIM processing.
 * @param raim_hfom [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.
 * @param raim_vfom [cm] Vertical expected accuracy using satellites successfully validated using RAIM.
 * @param corrections_quality  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.
 * @param system_status_summary  An abstract value representing the overall status of the receiver, or 255 if not available.
 * @param gnss_signal_quality  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.
 * @param post_processing_quality  An abstract value representing the estimated PPK quality, or 255 if not available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_integrity_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN];
    _mav_put_uint32_t(buf, 0, system_errors);
    _mav_put_uint16_t(buf, 4, raim_hfom);
    _mav_put_uint16_t(buf, 6, raim_vfom);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint8_t(buf, 9, authentication_state);
    _mav_put_uint8_t(buf, 10, jamming_state);
    _mav_put_uint8_t(buf, 11, spoofing_state);
    _mav_put_uint8_t(buf, 12, raim_state);
    _mav_put_uint8_t(buf, 13, corrections_quality);
    _mav_put_uint8_t(buf, 14, system_status_summary);
    _mav_put_uint8_t(buf, 15, gnss_signal_quality);
    _mav_put_uint8_t(buf, 16, post_processing_quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#else
    mavlink_gnss_integrity_t packet;
    packet.system_errors = system_errors;
    packet.raim_hfom = raim_hfom;
    packet.raim_vfom = raim_vfom;
    packet.id = id;
    packet.authentication_state = authentication_state;
    packet.jamming_state = jamming_state;
    packet.spoofing_state = spoofing_state;
    packet.raim_state = raim_state;
    packet.corrections_quality = corrections_quality;
    packet.system_status_summary = system_status_summary;
    packet.gnss_signal_quality = gnss_signal_quality;
    packet.post_processing_quality = post_processing_quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS_INTEGRITY;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
}

/**
 * @brief Pack a gnss_integrity message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param id  GNSS receiver id. Must match instance ids of other messages from same receiver.
 * @param system_errors  Errors in the GPS system.
 * @param authentication_state  Signal authentication state of the GPS system.
 * @param jamming_state  Signal jamming state of the GPS system.
 * @param spoofing_state  Signal spoofing state of the GPS system.
 * @param raim_state  The state of the RAIM processing.
 * @param raim_hfom [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.
 * @param raim_vfom [cm] Vertical expected accuracy using satellites successfully validated using RAIM.
 * @param corrections_quality  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.
 * @param system_status_summary  An abstract value representing the overall status of the receiver, or 255 if not available.
 * @param gnss_signal_quality  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.
 * @param post_processing_quality  An abstract value representing the estimated PPK quality, or 255 if not available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_integrity_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN];
    _mav_put_uint32_t(buf, 0, system_errors);
    _mav_put_uint16_t(buf, 4, raim_hfom);
    _mav_put_uint16_t(buf, 6, raim_vfom);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint8_t(buf, 9, authentication_state);
    _mav_put_uint8_t(buf, 10, jamming_state);
    _mav_put_uint8_t(buf, 11, spoofing_state);
    _mav_put_uint8_t(buf, 12, raim_state);
    _mav_put_uint8_t(buf, 13, corrections_quality);
    _mav_put_uint8_t(buf, 14, system_status_summary);
    _mav_put_uint8_t(buf, 15, gnss_signal_quality);
    _mav_put_uint8_t(buf, 16, post_processing_quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#else
    mavlink_gnss_integrity_t packet;
    packet.system_errors = system_errors;
    packet.raim_hfom = raim_hfom;
    packet.raim_vfom = raim_vfom;
    packet.id = id;
    packet.authentication_state = authentication_state;
    packet.jamming_state = jamming_state;
    packet.spoofing_state = spoofing_state;
    packet.raim_state = raim_state;
    packet.corrections_quality = corrections_quality;
    packet.system_status_summary = system_status_summary;
    packet.gnss_signal_quality = gnss_signal_quality;
    packet.post_processing_quality = post_processing_quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS_INTEGRITY;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#endif
}

/**
 * @brief Pack a gnss_integrity message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param id  GNSS receiver id. Must match instance ids of other messages from same receiver.
 * @param system_errors  Errors in the GPS system.
 * @param authentication_state  Signal authentication state of the GPS system.
 * @param jamming_state  Signal jamming state of the GPS system.
 * @param spoofing_state  Signal spoofing state of the GPS system.
 * @param raim_state  The state of the RAIM processing.
 * @param raim_hfom [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.
 * @param raim_vfom [cm] Vertical expected accuracy using satellites successfully validated using RAIM.
 * @param corrections_quality  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.
 * @param system_status_summary  An abstract value representing the overall status of the receiver, or 255 if not available.
 * @param gnss_signal_quality  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.
 * @param post_processing_quality  An abstract value representing the estimated PPK quality, or 255 if not available.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gnss_integrity_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t id,uint32_t system_errors,uint8_t authentication_state,uint8_t jamming_state,uint8_t spoofing_state,uint8_t raim_state,uint16_t raim_hfom,uint16_t raim_vfom,uint8_t corrections_quality,uint8_t system_status_summary,uint8_t gnss_signal_quality,uint8_t post_processing_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN];
    _mav_put_uint32_t(buf, 0, system_errors);
    _mav_put_uint16_t(buf, 4, raim_hfom);
    _mav_put_uint16_t(buf, 6, raim_vfom);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint8_t(buf, 9, authentication_state);
    _mav_put_uint8_t(buf, 10, jamming_state);
    _mav_put_uint8_t(buf, 11, spoofing_state);
    _mav_put_uint8_t(buf, 12, raim_state);
    _mav_put_uint8_t(buf, 13, corrections_quality);
    _mav_put_uint8_t(buf, 14, system_status_summary);
    _mav_put_uint8_t(buf, 15, gnss_signal_quality);
    _mav_put_uint8_t(buf, 16, post_processing_quality);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#else
    mavlink_gnss_integrity_t packet;
    packet.system_errors = system_errors;
    packet.raim_hfom = raim_hfom;
    packet.raim_vfom = raim_vfom;
    packet.id = id;
    packet.authentication_state = authentication_state;
    packet.jamming_state = jamming_state;
    packet.spoofing_state = spoofing_state;
    packet.raim_state = raim_state;
    packet.corrections_quality = corrections_quality;
    packet.system_status_summary = system_status_summary;
    packet.gnss_signal_quality = gnss_signal_quality;
    packet.post_processing_quality = post_processing_quality;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GNSS_INTEGRITY;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
}

/**
 * @brief Encode a gnss_integrity struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gnss_integrity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_integrity_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gnss_integrity_t* gnss_integrity)
{
    return mavlink_msg_gnss_integrity_pack(system_id, component_id, msg, gnss_integrity->id, gnss_integrity->system_errors, gnss_integrity->authentication_state, gnss_integrity->jamming_state, gnss_integrity->spoofing_state, gnss_integrity->raim_state, gnss_integrity->raim_hfom, gnss_integrity->raim_vfom, gnss_integrity->corrections_quality, gnss_integrity->system_status_summary, gnss_integrity->gnss_signal_quality, gnss_integrity->post_processing_quality);
}

/**
 * @brief Encode a gnss_integrity struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gnss_integrity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_integrity_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gnss_integrity_t* gnss_integrity)
{
    return mavlink_msg_gnss_integrity_pack_chan(system_id, component_id, chan, msg, gnss_integrity->id, gnss_integrity->system_errors, gnss_integrity->authentication_state, gnss_integrity->jamming_state, gnss_integrity->spoofing_state, gnss_integrity->raim_state, gnss_integrity->raim_hfom, gnss_integrity->raim_vfom, gnss_integrity->corrections_quality, gnss_integrity->system_status_summary, gnss_integrity->gnss_signal_quality, gnss_integrity->post_processing_quality);
}

/**
 * @brief Encode a gnss_integrity struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param gnss_integrity C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gnss_integrity_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_gnss_integrity_t* gnss_integrity)
{
    return mavlink_msg_gnss_integrity_pack_status(system_id, component_id, _status, msg,  gnss_integrity->id, gnss_integrity->system_errors, gnss_integrity->authentication_state, gnss_integrity->jamming_state, gnss_integrity->spoofing_state, gnss_integrity->raim_state, gnss_integrity->raim_hfom, gnss_integrity->raim_vfom, gnss_integrity->corrections_quality, gnss_integrity->system_status_summary, gnss_integrity->gnss_signal_quality, gnss_integrity->post_processing_quality);
}

/**
 * @brief Send a gnss_integrity message
 * @param chan MAVLink channel to send the message
 *
 * @param id  GNSS receiver id. Must match instance ids of other messages from same receiver.
 * @param system_errors  Errors in the GPS system.
 * @param authentication_state  Signal authentication state of the GPS system.
 * @param jamming_state  Signal jamming state of the GPS system.
 * @param spoofing_state  Signal spoofing state of the GPS system.
 * @param raim_state  The state of the RAIM processing.
 * @param raim_hfom [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.
 * @param raim_vfom [cm] Vertical expected accuracy using satellites successfully validated using RAIM.
 * @param corrections_quality  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.
 * @param system_status_summary  An abstract value representing the overall status of the receiver, or 255 if not available.
 * @param gnss_signal_quality  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.
 * @param post_processing_quality  An abstract value representing the estimated PPK quality, or 255 if not available.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gnss_integrity_send(mavlink_channel_t chan, uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN];
    _mav_put_uint32_t(buf, 0, system_errors);
    _mav_put_uint16_t(buf, 4, raim_hfom);
    _mav_put_uint16_t(buf, 6, raim_vfom);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint8_t(buf, 9, authentication_state);
    _mav_put_uint8_t(buf, 10, jamming_state);
    _mav_put_uint8_t(buf, 11, spoofing_state);
    _mav_put_uint8_t(buf, 12, raim_state);
    _mav_put_uint8_t(buf, 13, corrections_quality);
    _mav_put_uint8_t(buf, 14, system_status_summary);
    _mav_put_uint8_t(buf, 15, gnss_signal_quality);
    _mav_put_uint8_t(buf, 16, post_processing_quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS_INTEGRITY, buf, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#else
    mavlink_gnss_integrity_t packet;
    packet.system_errors = system_errors;
    packet.raim_hfom = raim_hfom;
    packet.raim_vfom = raim_vfom;
    packet.id = id;
    packet.authentication_state = authentication_state;
    packet.jamming_state = jamming_state;
    packet.spoofing_state = spoofing_state;
    packet.raim_state = raim_state;
    packet.corrections_quality = corrections_quality;
    packet.system_status_summary = system_status_summary;
    packet.gnss_signal_quality = gnss_signal_quality;
    packet.post_processing_quality = post_processing_quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS_INTEGRITY, (const char *)&packet, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#endif
}

/**
 * @brief Send a gnss_integrity message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gnss_integrity_send_struct(mavlink_channel_t chan, const mavlink_gnss_integrity_t* gnss_integrity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gnss_integrity_send(chan, gnss_integrity->id, gnss_integrity->system_errors, gnss_integrity->authentication_state, gnss_integrity->jamming_state, gnss_integrity->spoofing_state, gnss_integrity->raim_state, gnss_integrity->raim_hfom, gnss_integrity->raim_vfom, gnss_integrity->corrections_quality, gnss_integrity->system_status_summary, gnss_integrity->gnss_signal_quality, gnss_integrity->post_processing_quality);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS_INTEGRITY, (const char *)gnss_integrity, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#endif
}

#if MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gnss_integrity_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t id, uint32_t system_errors, uint8_t authentication_state, uint8_t jamming_state, uint8_t spoofing_state, uint8_t raim_state, uint16_t raim_hfom, uint16_t raim_vfom, uint8_t corrections_quality, uint8_t system_status_summary, uint8_t gnss_signal_quality, uint8_t post_processing_quality)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, system_errors);
    _mav_put_uint16_t(buf, 4, raim_hfom);
    _mav_put_uint16_t(buf, 6, raim_vfom);
    _mav_put_uint8_t(buf, 8, id);
    _mav_put_uint8_t(buf, 9, authentication_state);
    _mav_put_uint8_t(buf, 10, jamming_state);
    _mav_put_uint8_t(buf, 11, spoofing_state);
    _mav_put_uint8_t(buf, 12, raim_state);
    _mav_put_uint8_t(buf, 13, corrections_quality);
    _mav_put_uint8_t(buf, 14, system_status_summary);
    _mav_put_uint8_t(buf, 15, gnss_signal_quality);
    _mav_put_uint8_t(buf, 16, post_processing_quality);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS_INTEGRITY, buf, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#else
    mavlink_gnss_integrity_t *packet = (mavlink_gnss_integrity_t *)msgbuf;
    packet->system_errors = system_errors;
    packet->raim_hfom = raim_hfom;
    packet->raim_vfom = raim_vfom;
    packet->id = id;
    packet->authentication_state = authentication_state;
    packet->jamming_state = jamming_state;
    packet->spoofing_state = spoofing_state;
    packet->raim_state = raim_state;
    packet->corrections_quality = corrections_quality;
    packet->system_status_summary = system_status_summary;
    packet->gnss_signal_quality = gnss_signal_quality;
    packet->post_processing_quality = post_processing_quality;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GNSS_INTEGRITY, (const char *)packet, MAVLINK_MSG_ID_GNSS_INTEGRITY_MIN_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN, MAVLINK_MSG_ID_GNSS_INTEGRITY_CRC);
#endif
}
#endif

#endif

// MESSAGE GNSS_INTEGRITY UNPACKING


/**
 * @brief Get field id from gnss_integrity message
 *
 * @return  GNSS receiver id. Must match instance ids of other messages from same receiver.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field system_errors from gnss_integrity message
 *
 * @return  Errors in the GPS system.
 */
static inline uint32_t mavlink_msg_gnss_integrity_get_system_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field authentication_state from gnss_integrity message
 *
 * @return  Signal authentication state of the GPS system.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_authentication_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Get field jamming_state from gnss_integrity message
 *
 * @return  Signal jamming state of the GPS system.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_jamming_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field spoofing_state from gnss_integrity message
 *
 * @return  Signal spoofing state of the GPS system.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_spoofing_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field raim_state from gnss_integrity message
 *
 * @return  The state of the RAIM processing.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_raim_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field raim_hfom from gnss_integrity message
 *
 * @return [cm] Horizontal expected accuracy using satellites successfully validated using RAIM.
 */
static inline uint16_t mavlink_msg_gnss_integrity_get_raim_hfom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field raim_vfom from gnss_integrity message
 *
 * @return [cm] Vertical expected accuracy using satellites successfully validated using RAIM.
 */
static inline uint16_t mavlink_msg_gnss_integrity_get_raim_vfom(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field corrections_quality from gnss_integrity message
 *
 * @return  An abstract value representing the estimated quality of incoming corrections, or 255 if not available.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_corrections_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field system_status_summary from gnss_integrity message
 *
 * @return  An abstract value representing the overall status of the receiver, or 255 if not available.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_system_status_summary(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field gnss_signal_quality from gnss_integrity message
 *
 * @return  An abstract value representing the quality of incoming GNSS signals, or 255 if not available.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_gnss_signal_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field post_processing_quality from gnss_integrity message
 *
 * @return  An abstract value representing the estimated PPK quality, or 255 if not available.
 */
static inline uint8_t mavlink_msg_gnss_integrity_get_post_processing_quality(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a gnss_integrity message into a struct
 *
 * @param msg The message to decode
 * @param gnss_integrity C-struct to decode the message contents into
 */
static inline void mavlink_msg_gnss_integrity_decode(const mavlink_message_t* msg, mavlink_gnss_integrity_t* gnss_integrity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gnss_integrity->system_errors = mavlink_msg_gnss_integrity_get_system_errors(msg);
    gnss_integrity->raim_hfom = mavlink_msg_gnss_integrity_get_raim_hfom(msg);
    gnss_integrity->raim_vfom = mavlink_msg_gnss_integrity_get_raim_vfom(msg);
    gnss_integrity->id = mavlink_msg_gnss_integrity_get_id(msg);
    gnss_integrity->authentication_state = mavlink_msg_gnss_integrity_get_authentication_state(msg);
    gnss_integrity->jamming_state = mavlink_msg_gnss_integrity_get_jamming_state(msg);
    gnss_integrity->spoofing_state = mavlink_msg_gnss_integrity_get_spoofing_state(msg);
    gnss_integrity->raim_state = mavlink_msg_gnss_integrity_get_raim_state(msg);
    gnss_integrity->corrections_quality = mavlink_msg_gnss_integrity_get_corrections_quality(msg);
    gnss_integrity->system_status_summary = mavlink_msg_gnss_integrity_get_system_status_summary(msg);
    gnss_integrity->gnss_signal_quality = mavlink_msg_gnss_integrity_get_gnss_signal_quality(msg);
    gnss_integrity->post_processing_quality = mavlink_msg_gnss_integrity_get_post_processing_quality(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN? msg->len : MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN;
        memset(gnss_integrity, 0, MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN);
    memcpy(gnss_integrity, _MAV_PAYLOAD(msg), len);
#endif
}
