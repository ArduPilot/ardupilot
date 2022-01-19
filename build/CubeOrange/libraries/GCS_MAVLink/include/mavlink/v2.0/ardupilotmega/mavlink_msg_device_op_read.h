#pragma once
// MESSAGE DEVICE_OP_READ PACKING

#define MAVLINK_MSG_ID_DEVICE_OP_READ 11000


typedef struct __mavlink_device_op_read_t {
 uint32_t request_id; /*<  Request ID - copied to reply.*/
 uint8_t target_system; /*<  System ID.*/
 uint8_t target_component; /*<  Component ID.*/
 uint8_t bustype; /*<  The bus type.*/
 uint8_t bus; /*<  Bus number.*/
 uint8_t address; /*<  Bus address.*/
 char busname[40]; /*<  Name of device on bus (for SPI).*/
 uint8_t regstart; /*<  First register to read.*/
 uint8_t count; /*<  Count of registers to read.*/
 uint8_t bank; /*<  Bank number.*/
} mavlink_device_op_read_t;

#define MAVLINK_MSG_ID_DEVICE_OP_READ_LEN 52
#define MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN 51
#define MAVLINK_MSG_ID_11000_LEN 52
#define MAVLINK_MSG_ID_11000_MIN_LEN 51

#define MAVLINK_MSG_ID_DEVICE_OP_READ_CRC 134
#define MAVLINK_MSG_ID_11000_CRC 134

#define MAVLINK_MSG_DEVICE_OP_READ_FIELD_BUSNAME_LEN 40

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DEVICE_OP_READ { \
    11000, \
    "DEVICE_OP_READ", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_device_op_read_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_device_op_read_t, target_component) }, \
         { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_device_op_read_t, request_id) }, \
         { "bustype", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_device_op_read_t, bustype) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_device_op_read_t, bus) }, \
         { "address", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_device_op_read_t, address) }, \
         { "busname", NULL, MAVLINK_TYPE_CHAR, 40, 9, offsetof(mavlink_device_op_read_t, busname) }, \
         { "regstart", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_device_op_read_t, regstart) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_device_op_read_t, count) }, \
         { "bank", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_device_op_read_t, bank) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DEVICE_OP_READ { \
    "DEVICE_OP_READ", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_device_op_read_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_device_op_read_t, target_component) }, \
         { "request_id", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_device_op_read_t, request_id) }, \
         { "bustype", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_device_op_read_t, bustype) }, \
         { "bus", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_device_op_read_t, bus) }, \
         { "address", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_device_op_read_t, address) }, \
         { "busname", NULL, MAVLINK_TYPE_CHAR, 40, 9, offsetof(mavlink_device_op_read_t, busname) }, \
         { "regstart", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_device_op_read_t, regstart) }, \
         { "count", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_device_op_read_t, count) }, \
         { "bank", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_device_op_read_t, bank) }, \
         } \
}
#endif

/**
 * @brief Pack a device_op_read message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param bustype  The bus type.
 * @param bus  Bus number.
 * @param address  Bus address.
 * @param busname  Name of device on bus (for SPI).
 * @param regstart  First register to read.
 * @param count  Count of registers to read.
 * @param bank  Bank number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_op_read_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char *busname, uint8_t regstart, uint8_t count, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bustype);
    _mav_put_uint8_t(buf, 7, bus);
    _mav_put_uint8_t(buf, 8, address);
    _mav_put_uint8_t(buf, 49, regstart);
    _mav_put_uint8_t(buf, 50, count);
    _mav_put_uint8_t(buf, 51, bank);
    _mav_put_char_array(buf, 9, busname, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN);
#else
    mavlink_device_op_read_t packet;
    packet.request_id = request_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bustype = bustype;
    packet.bus = bus;
    packet.address = address;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.busname, busname, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_OP_READ;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
}

/**
 * @brief Pack a device_op_read message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param bustype  The bus type.
 * @param bus  Bus number.
 * @param address  Bus address.
 * @param busname  Name of device on bus (for SPI).
 * @param regstart  First register to read.
 * @param count  Count of registers to read.
 * @param bank  Bank number.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_device_op_read_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint32_t request_id,uint8_t bustype,uint8_t bus,uint8_t address,const char *busname,uint8_t regstart,uint8_t count,uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bustype);
    _mav_put_uint8_t(buf, 7, bus);
    _mav_put_uint8_t(buf, 8, address);
    _mav_put_uint8_t(buf, 49, regstart);
    _mav_put_uint8_t(buf, 50, count);
    _mav_put_uint8_t(buf, 51, bank);
    _mav_put_char_array(buf, 9, busname, 40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN);
#else
    mavlink_device_op_read_t packet;
    packet.request_id = request_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bustype = bustype;
    packet.bus = bus;
    packet.address = address;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.busname, busname, sizeof(char)*40);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DEVICE_OP_READ;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
}

/**
 * @brief Encode a device_op_read struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param device_op_read C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_op_read_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_device_op_read_t* device_op_read)
{
    return mavlink_msg_device_op_read_pack(system_id, component_id, msg, device_op_read->target_system, device_op_read->target_component, device_op_read->request_id, device_op_read->bustype, device_op_read->bus, device_op_read->address, device_op_read->busname, device_op_read->regstart, device_op_read->count, device_op_read->bank);
}

/**
 * @brief Encode a device_op_read struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param device_op_read C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_device_op_read_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_device_op_read_t* device_op_read)
{
    return mavlink_msg_device_op_read_pack_chan(system_id, component_id, chan, msg, device_op_read->target_system, device_op_read->target_component, device_op_read->request_id, device_op_read->bustype, device_op_read->bus, device_op_read->address, device_op_read->busname, device_op_read->regstart, device_op_read->count, device_op_read->bank);
}

/**
 * @brief Send a device_op_read message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID.
 * @param target_component  Component ID.
 * @param request_id  Request ID - copied to reply.
 * @param bustype  The bus type.
 * @param bus  Bus number.
 * @param address  Bus address.
 * @param busname  Name of device on bus (for SPI).
 * @param regstart  First register to read.
 * @param count  Count of registers to read.
 * @param bank  Bank number.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_device_op_read_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char *busname, uint8_t regstart, uint8_t count, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DEVICE_OP_READ_LEN];
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bustype);
    _mav_put_uint8_t(buf, 7, bus);
    _mav_put_uint8_t(buf, 8, address);
    _mav_put_uint8_t(buf, 49, regstart);
    _mav_put_uint8_t(buf, 50, count);
    _mav_put_uint8_t(buf, 51, bank);
    _mav_put_char_array(buf, 9, busname, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ, buf, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
#else
    mavlink_device_op_read_t packet;
    packet.request_id = request_id;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.bustype = bustype;
    packet.bus = bus;
    packet.address = address;
    packet.regstart = regstart;
    packet.count = count;
    packet.bank = bank;
    mav_array_memcpy(packet.busname, busname, sizeof(char)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ, (const char *)&packet, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
#endif
}

/**
 * @brief Send a device_op_read message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_device_op_read_send_struct(mavlink_channel_t chan, const mavlink_device_op_read_t* device_op_read)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_device_op_read_send(chan, device_op_read->target_system, device_op_read->target_component, device_op_read->request_id, device_op_read->bustype, device_op_read->bus, device_op_read->address, device_op_read->busname, device_op_read->regstart, device_op_read->count, device_op_read->bank);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ, (const char *)device_op_read, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
#endif
}

#if MAVLINK_MSG_ID_DEVICE_OP_READ_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_device_op_read_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint32_t request_id, uint8_t bustype, uint8_t bus, uint8_t address, const char *busname, uint8_t regstart, uint8_t count, uint8_t bank)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, request_id);
    _mav_put_uint8_t(buf, 4, target_system);
    _mav_put_uint8_t(buf, 5, target_component);
    _mav_put_uint8_t(buf, 6, bustype);
    _mav_put_uint8_t(buf, 7, bus);
    _mav_put_uint8_t(buf, 8, address);
    _mav_put_uint8_t(buf, 49, regstart);
    _mav_put_uint8_t(buf, 50, count);
    _mav_put_uint8_t(buf, 51, bank);
    _mav_put_char_array(buf, 9, busname, 40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ, buf, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
#else
    mavlink_device_op_read_t *packet = (mavlink_device_op_read_t *)msgbuf;
    packet->request_id = request_id;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->bustype = bustype;
    packet->bus = bus;
    packet->address = address;
    packet->regstart = regstart;
    packet->count = count;
    packet->bank = bank;
    mav_array_memcpy(packet->busname, busname, sizeof(char)*40);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DEVICE_OP_READ, (const char *)packet, MAVLINK_MSG_ID_DEVICE_OP_READ_MIN_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN, MAVLINK_MSG_ID_DEVICE_OP_READ_CRC);
#endif
}
#endif

#endif

// MESSAGE DEVICE_OP_READ UNPACKING


/**
 * @brief Get field target_system from device_op_read message
 *
 * @return  System ID.
 */
static inline uint8_t mavlink_msg_device_op_read_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from device_op_read message
 *
 * @return  Component ID.
 */
static inline uint8_t mavlink_msg_device_op_read_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field request_id from device_op_read message
 *
 * @return  Request ID - copied to reply.
 */
static inline uint32_t mavlink_msg_device_op_read_get_request_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field bustype from device_op_read message
 *
 * @return  The bus type.
 */
static inline uint8_t mavlink_msg_device_op_read_get_bustype(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field bus from device_op_read message
 *
 * @return  Bus number.
 */
static inline uint8_t mavlink_msg_device_op_read_get_bus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field address from device_op_read message
 *
 * @return  Bus address.
 */
static inline uint8_t mavlink_msg_device_op_read_get_address(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field busname from device_op_read message
 *
 * @return  Name of device on bus (for SPI).
 */
static inline uint16_t mavlink_msg_device_op_read_get_busname(const mavlink_message_t* msg, char *busname)
{
    return _MAV_RETURN_char_array(msg, busname, 40,  9);
}

/**
 * @brief Get field regstart from device_op_read message
 *
 * @return  First register to read.
 */
static inline uint8_t mavlink_msg_device_op_read_get_regstart(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field count from device_op_read message
 *
 * @return  Count of registers to read.
 */
static inline uint8_t mavlink_msg_device_op_read_get_count(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field bank from device_op_read message
 *
 * @return  Bank number.
 */
static inline uint8_t mavlink_msg_device_op_read_get_bank(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Decode a device_op_read message into a struct
 *
 * @param msg The message to decode
 * @param device_op_read C-struct to decode the message contents into
 */
static inline void mavlink_msg_device_op_read_decode(const mavlink_message_t* msg, mavlink_device_op_read_t* device_op_read)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    device_op_read->request_id = mavlink_msg_device_op_read_get_request_id(msg);
    device_op_read->target_system = mavlink_msg_device_op_read_get_target_system(msg);
    device_op_read->target_component = mavlink_msg_device_op_read_get_target_component(msg);
    device_op_read->bustype = mavlink_msg_device_op_read_get_bustype(msg);
    device_op_read->bus = mavlink_msg_device_op_read_get_bus(msg);
    device_op_read->address = mavlink_msg_device_op_read_get_address(msg);
    mavlink_msg_device_op_read_get_busname(msg, device_op_read->busname);
    device_op_read->regstart = mavlink_msg_device_op_read_get_regstart(msg);
    device_op_read->count = mavlink_msg_device_op_read_get_count(msg);
    device_op_read->bank = mavlink_msg_device_op_read_get_bank(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DEVICE_OP_READ_LEN? msg->len : MAVLINK_MSG_ID_DEVICE_OP_READ_LEN;
        memset(device_op_read, 0, MAVLINK_MSG_ID_DEVICE_OP_READ_LEN);
    memcpy(device_op_read, _MAV_PAYLOAD(msg), len);
#endif
}
