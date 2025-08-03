#pragma once
// MESSAGE RESOURCE_REQUEST PACKING

#define MAVLINK_MSG_ID_RESOURCE_REQUEST 142


typedef struct __mavlink_resource_request_t {
 uint8_t request_id; /*<  Request ID. This ID should be re-used when sending back URI contents*/
 uint8_t uri_type; /*<  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary*/
 uint8_t uri[120]; /*<  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)*/
 uint8_t transfer_type; /*<  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.*/
 uint8_t storage[120]; /*<  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).*/
} mavlink_resource_request_t;

#define MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN 243
#define MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN 243
#define MAVLINK_MSG_ID_142_LEN 243
#define MAVLINK_MSG_ID_142_MIN_LEN 243

#define MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC 72
#define MAVLINK_MSG_ID_142_CRC 72

#define MAVLINK_MSG_RESOURCE_REQUEST_FIELD_URI_LEN 120
#define MAVLINK_MSG_RESOURCE_REQUEST_FIELD_STORAGE_LEN 120

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST { \
    142, \
    "RESOURCE_REQUEST", \
    5, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_resource_request_t, request_id) }, \
         { "uri_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_resource_request_t, uri_type) }, \
         { "uri", NULL, MAVLINK_TYPE_UINT8_T, 120, 2, offsetof(mavlink_resource_request_t, uri) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 122, offsetof(mavlink_resource_request_t, transfer_type) }, \
         { "storage", NULL, MAVLINK_TYPE_UINT8_T, 120, 123, offsetof(mavlink_resource_request_t, storage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RESOURCE_REQUEST { \
    "RESOURCE_REQUEST", \
    5, \
    {  { "request_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_resource_request_t, request_id) }, \
         { "uri_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_resource_request_t, uri_type) }, \
         { "uri", NULL, MAVLINK_TYPE_UINT8_T, 120, 2, offsetof(mavlink_resource_request_t, uri) }, \
         { "transfer_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 122, offsetof(mavlink_resource_request_t, transfer_type) }, \
         { "storage", NULL, MAVLINK_TYPE_UINT8_T, 120, 123, offsetof(mavlink_resource_request_t, storage) }, \
         } \
}
#endif

/**
 * @brief Pack a resource_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_id  Request ID. This ID should be re-used when sending back URI contents
 * @param uri_type  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
 * @param uri  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
 * @param transfer_type  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
 * @param storage  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_resource_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t request_id, uint8_t uri_type, const uint8_t *uri, uint8_t transfer_type, const uint8_t *storage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 1, uri_type);
    _mav_put_uint8_t(buf, 122, transfer_type);
    _mav_put_uint8_t_array(buf, 2, uri, 120);
    _mav_put_uint8_t_array(buf, 123, storage, 120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#else
    mavlink_resource_request_t packet;
    packet.request_id = request_id;
    packet.uri_type = uri_type;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.uri, uri, sizeof(uint8_t)*120);
    mav_array_memcpy(packet.storage, storage, sizeof(uint8_t)*120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESOURCE_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
}

/**
 * @brief Pack a resource_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param request_id  Request ID. This ID should be re-used when sending back URI contents
 * @param uri_type  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
 * @param uri  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
 * @param transfer_type  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
 * @param storage  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_resource_request_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t request_id, uint8_t uri_type, const uint8_t *uri, uint8_t transfer_type, const uint8_t *storage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 1, uri_type);
    _mav_put_uint8_t(buf, 122, transfer_type);
    _mav_put_uint8_t_array(buf, 2, uri, 120);
    _mav_put_uint8_t_array(buf, 123, storage, 120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#else
    mavlink_resource_request_t packet;
    packet.request_id = request_id;
    packet.uri_type = uri_type;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.uri, uri, sizeof(uint8_t)*120);
    mav_array_memcpy(packet.storage, storage, sizeof(uint8_t)*120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESOURCE_REQUEST;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#endif
}

/**
 * @brief Pack a resource_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param request_id  Request ID. This ID should be re-used when sending back URI contents
 * @param uri_type  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
 * @param uri  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
 * @param transfer_type  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
 * @param storage  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_resource_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t request_id,uint8_t uri_type,const uint8_t *uri,uint8_t transfer_type,const uint8_t *storage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 1, uri_type);
    _mav_put_uint8_t(buf, 122, transfer_type);
    _mav_put_uint8_t_array(buf, 2, uri, 120);
    _mav_put_uint8_t_array(buf, 123, storage, 120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#else
    mavlink_resource_request_t packet;
    packet.request_id = request_id;
    packet.uri_type = uri_type;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.uri, uri, sizeof(uint8_t)*120);
    mav_array_memcpy(packet.storage, storage, sizeof(uint8_t)*120);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RESOURCE_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
}

/**
 * @brief Encode a resource_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param resource_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_resource_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_resource_request_t* resource_request)
{
    return mavlink_msg_resource_request_pack(system_id, component_id, msg, resource_request->request_id, resource_request->uri_type, resource_request->uri, resource_request->transfer_type, resource_request->storage);
}

/**
 * @brief Encode a resource_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param resource_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_resource_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_resource_request_t* resource_request)
{
    return mavlink_msg_resource_request_pack_chan(system_id, component_id, chan, msg, resource_request->request_id, resource_request->uri_type, resource_request->uri, resource_request->transfer_type, resource_request->storage);
}

/**
 * @brief Encode a resource_request struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param resource_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_resource_request_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_resource_request_t* resource_request)
{
    return mavlink_msg_resource_request_pack_status(system_id, component_id, _status, msg,  resource_request->request_id, resource_request->uri_type, resource_request->uri, resource_request->transfer_type, resource_request->storage);
}

/**
 * @brief Send a resource_request message
 * @param chan MAVLink channel to send the message
 *
 * @param request_id  Request ID. This ID should be re-used when sending back URI contents
 * @param uri_type  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
 * @param uri  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
 * @param transfer_type  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
 * @param storage  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_resource_request_send(mavlink_channel_t chan, uint8_t request_id, uint8_t uri_type, const uint8_t *uri, uint8_t transfer_type, const uint8_t *storage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 1, uri_type);
    _mav_put_uint8_t(buf, 122, transfer_type);
    _mav_put_uint8_t_array(buf, 2, uri, 120);
    _mav_put_uint8_t_array(buf, 123, storage, 120);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESOURCE_REQUEST, buf, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#else
    mavlink_resource_request_t packet;
    packet.request_id = request_id;
    packet.uri_type = uri_type;
    packet.transfer_type = transfer_type;
    mav_array_memcpy(packet.uri, uri, sizeof(uint8_t)*120);
    mav_array_memcpy(packet.storage, storage, sizeof(uint8_t)*120);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESOURCE_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#endif
}

/**
 * @brief Send a resource_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_resource_request_send_struct(mavlink_channel_t chan, const mavlink_resource_request_t* resource_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_resource_request_send(chan, resource_request->request_id, resource_request->uri_type, resource_request->uri, resource_request->transfer_type, resource_request->storage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESOURCE_REQUEST, (const char *)resource_request, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_resource_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t request_id, uint8_t uri_type, const uint8_t *uri, uint8_t transfer_type, const uint8_t *storage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 1, uri_type);
    _mav_put_uint8_t(buf, 122, transfer_type);
    _mav_put_uint8_t_array(buf, 2, uri, 120);
    _mav_put_uint8_t_array(buf, 123, storage, 120);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESOURCE_REQUEST, buf, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#else
    mavlink_resource_request_t *packet = (mavlink_resource_request_t *)msgbuf;
    packet->request_id = request_id;
    packet->uri_type = uri_type;
    packet->transfer_type = transfer_type;
    mav_array_memcpy(packet->uri, uri, sizeof(uint8_t)*120);
    mav_array_memcpy(packet->storage, storage, sizeof(uint8_t)*120);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RESOURCE_REQUEST, (const char *)packet, MAVLINK_MSG_ID_RESOURCE_REQUEST_MIN_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN, MAVLINK_MSG_ID_RESOURCE_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE RESOURCE_REQUEST UNPACKING


/**
 * @brief Get field request_id from resource_request message
 *
 * @return  Request ID. This ID should be re-used when sending back URI contents
 */
static inline uint8_t mavlink_msg_resource_request_get_request_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field uri_type from resource_request message
 *
 * @return  The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
 */
static inline uint8_t mavlink_msg_resource_request_get_uri_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field uri from resource_request message
 *
 * @return  The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
 */
static inline uint16_t mavlink_msg_resource_request_get_uri(const mavlink_message_t* msg, uint8_t *uri)
{
    return _MAV_RETURN_uint8_t_array(msg, uri, 120,  2);
}

/**
 * @brief Get field transfer_type from resource_request message
 *
 * @return  The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
 */
static inline uint8_t mavlink_msg_resource_request_get_transfer_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  122);
}

/**
 * @brief Get field storage from resource_request message
 *
 * @return  The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
 */
static inline uint16_t mavlink_msg_resource_request_get_storage(const mavlink_message_t* msg, uint8_t *storage)
{
    return _MAV_RETURN_uint8_t_array(msg, storage, 120,  123);
}

/**
 * @brief Decode a resource_request message into a struct
 *
 * @param msg The message to decode
 * @param resource_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_resource_request_decode(const mavlink_message_t* msg, mavlink_resource_request_t* resource_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    resource_request->request_id = mavlink_msg_resource_request_get_request_id(msg);
    resource_request->uri_type = mavlink_msg_resource_request_get_uri_type(msg);
    mavlink_msg_resource_request_get_uri(msg, resource_request->uri);
    resource_request->transfer_type = mavlink_msg_resource_request_get_transfer_type(msg);
    mavlink_msg_resource_request_get_storage(msg, resource_request->storage);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN;
        memset(resource_request, 0, MAVLINK_MSG_ID_RESOURCE_REQUEST_LEN);
    memcpy(resource_request, _MAV_PAYLOAD(msg), len);
#endif
}
