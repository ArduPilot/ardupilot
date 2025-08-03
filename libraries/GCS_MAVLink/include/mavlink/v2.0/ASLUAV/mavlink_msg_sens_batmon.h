#pragma once
// MESSAGE SENS_BATMON PACKING

#define MAVLINK_MSG_ID_SENS_BATMON 8010


typedef struct __mavlink_sens_batmon_t {
 uint64_t batmon_timestamp; /*< [us] Time since system start*/
 float temperature; /*< [degC] Battery pack temperature*/
 uint32_t safetystatus; /*<  Battery monitor safetystatus report bits in Hex*/
 uint32_t operationstatus; /*<  Battery monitor operation status report bits in Hex*/
 uint16_t voltage; /*< [mV] Battery pack voltage*/
 int16_t current; /*< [mA] Battery pack current*/
 uint16_t batterystatus; /*<  Battery monitor status report bits in Hex*/
 uint16_t serialnumber; /*<  Battery monitor serial number in Hex*/
 uint16_t cellvoltage1; /*< [mV] Battery pack cell 1 voltage*/
 uint16_t cellvoltage2; /*< [mV] Battery pack cell 2 voltage*/
 uint16_t cellvoltage3; /*< [mV] Battery pack cell 3 voltage*/
 uint16_t cellvoltage4; /*< [mV] Battery pack cell 4 voltage*/
 uint16_t cellvoltage5; /*< [mV] Battery pack cell 5 voltage*/
 uint16_t cellvoltage6; /*< [mV] Battery pack cell 6 voltage*/
 uint8_t SoC; /*<  Battery pack state-of-charge*/
} mavlink_sens_batmon_t;

#define MAVLINK_MSG_ID_SENS_BATMON_LEN 41
#define MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN 41
#define MAVLINK_MSG_ID_8010_LEN 41
#define MAVLINK_MSG_ID_8010_MIN_LEN 41

#define MAVLINK_MSG_ID_SENS_BATMON_CRC 155
#define MAVLINK_MSG_ID_8010_CRC 155



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SENS_BATMON { \
    8010, \
    "SENS_BATMON", \
    15, \
    {  { "batmon_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_batmon_t, batmon_timestamp) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_batmon_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sens_batmon_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_sens_batmon_t, current) }, \
         { "SoC", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sens_batmon_t, SoC) }, \
         { "batterystatus", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_sens_batmon_t, batterystatus) }, \
         { "serialnumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_sens_batmon_t, serialnumber) }, \
         { "safetystatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sens_batmon_t, safetystatus) }, \
         { "operationstatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sens_batmon_t, operationstatus) }, \
         { "cellvoltage1", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_sens_batmon_t, cellvoltage1) }, \
         { "cellvoltage2", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_sens_batmon_t, cellvoltage2) }, \
         { "cellvoltage3", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sens_batmon_t, cellvoltage3) }, \
         { "cellvoltage4", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sens_batmon_t, cellvoltage4) }, \
         { "cellvoltage5", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sens_batmon_t, cellvoltage5) }, \
         { "cellvoltage6", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_sens_batmon_t, cellvoltage6) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SENS_BATMON { \
    "SENS_BATMON", \
    15, \
    {  { "batmon_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sens_batmon_t, batmon_timestamp) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sens_batmon_t, temperature) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sens_batmon_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_sens_batmon_t, current) }, \
         { "SoC", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sens_batmon_t, SoC) }, \
         { "batterystatus", NULL, MAVLINK_TYPE_UINT16_T, 0, 24, offsetof(mavlink_sens_batmon_t, batterystatus) }, \
         { "serialnumber", NULL, MAVLINK_TYPE_UINT16_T, 0, 26, offsetof(mavlink_sens_batmon_t, serialnumber) }, \
         { "safetystatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sens_batmon_t, safetystatus) }, \
         { "operationstatus", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sens_batmon_t, operationstatus) }, \
         { "cellvoltage1", NULL, MAVLINK_TYPE_UINT16_T, 0, 28, offsetof(mavlink_sens_batmon_t, cellvoltage1) }, \
         { "cellvoltage2", NULL, MAVLINK_TYPE_UINT16_T, 0, 30, offsetof(mavlink_sens_batmon_t, cellvoltage2) }, \
         { "cellvoltage3", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sens_batmon_t, cellvoltage3) }, \
         { "cellvoltage4", NULL, MAVLINK_TYPE_UINT16_T, 0, 34, offsetof(mavlink_sens_batmon_t, cellvoltage4) }, \
         { "cellvoltage5", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_sens_batmon_t, cellvoltage5) }, \
         { "cellvoltage6", NULL, MAVLINK_TYPE_UINT16_T, 0, 38, offsetof(mavlink_sens_batmon_t, cellvoltage6) }, \
         } \
}
#endif

/**
 * @brief Pack a sens_batmon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param batmon_timestamp [us] Time since system start
 * @param temperature [degC] Battery pack temperature
 * @param voltage [mV] Battery pack voltage
 * @param current [mA] Battery pack current
 * @param SoC  Battery pack state-of-charge
 * @param batterystatus  Battery monitor status report bits in Hex
 * @param serialnumber  Battery monitor serial number in Hex
 * @param safetystatus  Battery monitor safetystatus report bits in Hex
 * @param operationstatus  Battery monitor operation status report bits in Hex
 * @param cellvoltage1 [mV] Battery pack cell 1 voltage
 * @param cellvoltage2 [mV] Battery pack cell 2 voltage
 * @param cellvoltage3 [mV] Battery pack cell 3 voltage
 * @param cellvoltage4 [mV] Battery pack cell 4 voltage
 * @param cellvoltage5 [mV] Battery pack cell 5 voltage
 * @param cellvoltage6 [mV] Battery pack cell 6 voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_batmon_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
    _mav_put_uint64_t(buf, 0, batmon_timestamp);
    _mav_put_float(buf, 8, temperature);
    _mav_put_uint32_t(buf, 12, safetystatus);
    _mav_put_uint32_t(buf, 16, operationstatus);
    _mav_put_uint16_t(buf, 20, voltage);
    _mav_put_int16_t(buf, 22, current);
    _mav_put_uint16_t(buf, 24, batterystatus);
    _mav_put_uint16_t(buf, 26, serialnumber);
    _mav_put_uint16_t(buf, 28, cellvoltage1);
    _mav_put_uint16_t(buf, 30, cellvoltage2);
    _mav_put_uint16_t(buf, 32, cellvoltage3);
    _mav_put_uint16_t(buf, 34, cellvoltage4);
    _mav_put_uint16_t(buf, 36, cellvoltage5);
    _mav_put_uint16_t(buf, 38, cellvoltage6);
    _mav_put_uint8_t(buf, 40, SoC);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#else
    mavlink_sens_batmon_t packet;
    packet.batmon_timestamp = batmon_timestamp;
    packet.temperature = temperature;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.serialnumber = serialnumber;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.SoC = SoC;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_BATMON;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
}

/**
 * @brief Pack a sens_batmon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param batmon_timestamp [us] Time since system start
 * @param temperature [degC] Battery pack temperature
 * @param voltage [mV] Battery pack voltage
 * @param current [mA] Battery pack current
 * @param SoC  Battery pack state-of-charge
 * @param batterystatus  Battery monitor status report bits in Hex
 * @param serialnumber  Battery monitor serial number in Hex
 * @param safetystatus  Battery monitor safetystatus report bits in Hex
 * @param operationstatus  Battery monitor operation status report bits in Hex
 * @param cellvoltage1 [mV] Battery pack cell 1 voltage
 * @param cellvoltage2 [mV] Battery pack cell 2 voltage
 * @param cellvoltage3 [mV] Battery pack cell 3 voltage
 * @param cellvoltage4 [mV] Battery pack cell 4 voltage
 * @param cellvoltage5 [mV] Battery pack cell 5 voltage
 * @param cellvoltage6 [mV] Battery pack cell 6 voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_batmon_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
    _mav_put_uint64_t(buf, 0, batmon_timestamp);
    _mav_put_float(buf, 8, temperature);
    _mav_put_uint32_t(buf, 12, safetystatus);
    _mav_put_uint32_t(buf, 16, operationstatus);
    _mav_put_uint16_t(buf, 20, voltage);
    _mav_put_int16_t(buf, 22, current);
    _mav_put_uint16_t(buf, 24, batterystatus);
    _mav_put_uint16_t(buf, 26, serialnumber);
    _mav_put_uint16_t(buf, 28, cellvoltage1);
    _mav_put_uint16_t(buf, 30, cellvoltage2);
    _mav_put_uint16_t(buf, 32, cellvoltage3);
    _mav_put_uint16_t(buf, 34, cellvoltage4);
    _mav_put_uint16_t(buf, 36, cellvoltage5);
    _mav_put_uint16_t(buf, 38, cellvoltage6);
    _mav_put_uint8_t(buf, 40, SoC);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#else
    mavlink_sens_batmon_t packet;
    packet.batmon_timestamp = batmon_timestamp;
    packet.temperature = temperature;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.serialnumber = serialnumber;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.SoC = SoC;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_BATMON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif
}

/**
 * @brief Pack a sens_batmon message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param batmon_timestamp [us] Time since system start
 * @param temperature [degC] Battery pack temperature
 * @param voltage [mV] Battery pack voltage
 * @param current [mA] Battery pack current
 * @param SoC  Battery pack state-of-charge
 * @param batterystatus  Battery monitor status report bits in Hex
 * @param serialnumber  Battery monitor serial number in Hex
 * @param safetystatus  Battery monitor safetystatus report bits in Hex
 * @param operationstatus  Battery monitor operation status report bits in Hex
 * @param cellvoltage1 [mV] Battery pack cell 1 voltage
 * @param cellvoltage2 [mV] Battery pack cell 2 voltage
 * @param cellvoltage3 [mV] Battery pack cell 3 voltage
 * @param cellvoltage4 [mV] Battery pack cell 4 voltage
 * @param cellvoltage5 [mV] Battery pack cell 5 voltage
 * @param cellvoltage6 [mV] Battery pack cell 6 voltage
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sens_batmon_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t batmon_timestamp,float temperature,uint16_t voltage,int16_t current,uint8_t SoC,uint16_t batterystatus,uint16_t serialnumber,uint32_t safetystatus,uint32_t operationstatus,uint16_t cellvoltage1,uint16_t cellvoltage2,uint16_t cellvoltage3,uint16_t cellvoltage4,uint16_t cellvoltage5,uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
    _mav_put_uint64_t(buf, 0, batmon_timestamp);
    _mav_put_float(buf, 8, temperature);
    _mav_put_uint32_t(buf, 12, safetystatus);
    _mav_put_uint32_t(buf, 16, operationstatus);
    _mav_put_uint16_t(buf, 20, voltage);
    _mav_put_int16_t(buf, 22, current);
    _mav_put_uint16_t(buf, 24, batterystatus);
    _mav_put_uint16_t(buf, 26, serialnumber);
    _mav_put_uint16_t(buf, 28, cellvoltage1);
    _mav_put_uint16_t(buf, 30, cellvoltage2);
    _mav_put_uint16_t(buf, 32, cellvoltage3);
    _mav_put_uint16_t(buf, 34, cellvoltage4);
    _mav_put_uint16_t(buf, 36, cellvoltage5);
    _mav_put_uint16_t(buf, 38, cellvoltage6);
    _mav_put_uint8_t(buf, 40, SoC);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#else
    mavlink_sens_batmon_t packet;
    packet.batmon_timestamp = batmon_timestamp;
    packet.temperature = temperature;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.serialnumber = serialnumber;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.SoC = SoC;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SENS_BATMON_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SENS_BATMON;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
}

/**
 * @brief Encode a sens_batmon struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sens_batmon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_batmon_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sens_batmon_t* sens_batmon)
{
    return mavlink_msg_sens_batmon_pack(system_id, component_id, msg, sens_batmon->batmon_timestamp, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->SoC, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->safetystatus, sens_batmon->operationstatus, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
}

/**
 * @brief Encode a sens_batmon struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sens_batmon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_batmon_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sens_batmon_t* sens_batmon)
{
    return mavlink_msg_sens_batmon_pack_chan(system_id, component_id, chan, msg, sens_batmon->batmon_timestamp, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->SoC, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->safetystatus, sens_batmon->operationstatus, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
}

/**
 * @brief Encode a sens_batmon struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param sens_batmon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sens_batmon_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_sens_batmon_t* sens_batmon)
{
    return mavlink_msg_sens_batmon_pack_status(system_id, component_id, _status, msg,  sens_batmon->batmon_timestamp, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->SoC, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->safetystatus, sens_batmon->operationstatus, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
}

/**
 * @brief Send a sens_batmon message
 * @param chan MAVLink channel to send the message
 *
 * @param batmon_timestamp [us] Time since system start
 * @param temperature [degC] Battery pack temperature
 * @param voltage [mV] Battery pack voltage
 * @param current [mA] Battery pack current
 * @param SoC  Battery pack state-of-charge
 * @param batterystatus  Battery monitor status report bits in Hex
 * @param serialnumber  Battery monitor serial number in Hex
 * @param safetystatus  Battery monitor safetystatus report bits in Hex
 * @param operationstatus  Battery monitor operation status report bits in Hex
 * @param cellvoltage1 [mV] Battery pack cell 1 voltage
 * @param cellvoltage2 [mV] Battery pack cell 2 voltage
 * @param cellvoltage3 [mV] Battery pack cell 3 voltage
 * @param cellvoltage4 [mV] Battery pack cell 4 voltage
 * @param cellvoltage5 [mV] Battery pack cell 5 voltage
 * @param cellvoltage6 [mV] Battery pack cell 6 voltage
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sens_batmon_send(mavlink_channel_t chan, uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SENS_BATMON_LEN];
    _mav_put_uint64_t(buf, 0, batmon_timestamp);
    _mav_put_float(buf, 8, temperature);
    _mav_put_uint32_t(buf, 12, safetystatus);
    _mav_put_uint32_t(buf, 16, operationstatus);
    _mav_put_uint16_t(buf, 20, voltage);
    _mav_put_int16_t(buf, 22, current);
    _mav_put_uint16_t(buf, 24, batterystatus);
    _mav_put_uint16_t(buf, 26, serialnumber);
    _mav_put_uint16_t(buf, 28, cellvoltage1);
    _mav_put_uint16_t(buf, 30, cellvoltage2);
    _mav_put_uint16_t(buf, 32, cellvoltage3);
    _mav_put_uint16_t(buf, 34, cellvoltage4);
    _mav_put_uint16_t(buf, 36, cellvoltage5);
    _mav_put_uint16_t(buf, 38, cellvoltage6);
    _mav_put_uint8_t(buf, 40, SoC);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    mavlink_sens_batmon_t packet;
    packet.batmon_timestamp = batmon_timestamp;
    packet.temperature = temperature;
    packet.safetystatus = safetystatus;
    packet.operationstatus = operationstatus;
    packet.voltage = voltage;
    packet.current = current;
    packet.batterystatus = batterystatus;
    packet.serialnumber = serialnumber;
    packet.cellvoltage1 = cellvoltage1;
    packet.cellvoltage2 = cellvoltage2;
    packet.cellvoltage3 = cellvoltage3;
    packet.cellvoltage4 = cellvoltage4;
    packet.cellvoltage5 = cellvoltage5;
    packet.cellvoltage6 = cellvoltage6;
    packet.SoC = SoC;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)&packet, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#endif
}

/**
 * @brief Send a sens_batmon message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sens_batmon_send_struct(mavlink_channel_t chan, const mavlink_sens_batmon_t* sens_batmon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sens_batmon_send(chan, sens_batmon->batmon_timestamp, sens_batmon->temperature, sens_batmon->voltage, sens_batmon->current, sens_batmon->SoC, sens_batmon->batterystatus, sens_batmon->serialnumber, sens_batmon->safetystatus, sens_batmon->operationstatus, sens_batmon->cellvoltage1, sens_batmon->cellvoltage2, sens_batmon->cellvoltage3, sens_batmon->cellvoltage4, sens_batmon->cellvoltage5, sens_batmon->cellvoltage6);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)sens_batmon, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#endif
}

#if MAVLINK_MSG_ID_SENS_BATMON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sens_batmon_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t batmon_timestamp, float temperature, uint16_t voltage, int16_t current, uint8_t SoC, uint16_t batterystatus, uint16_t serialnumber, uint32_t safetystatus, uint32_t operationstatus, uint16_t cellvoltage1, uint16_t cellvoltage2, uint16_t cellvoltage3, uint16_t cellvoltage4, uint16_t cellvoltage5, uint16_t cellvoltage6)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, batmon_timestamp);
    _mav_put_float(buf, 8, temperature);
    _mav_put_uint32_t(buf, 12, safetystatus);
    _mav_put_uint32_t(buf, 16, operationstatus);
    _mav_put_uint16_t(buf, 20, voltage);
    _mav_put_int16_t(buf, 22, current);
    _mav_put_uint16_t(buf, 24, batterystatus);
    _mav_put_uint16_t(buf, 26, serialnumber);
    _mav_put_uint16_t(buf, 28, cellvoltage1);
    _mav_put_uint16_t(buf, 30, cellvoltage2);
    _mav_put_uint16_t(buf, 32, cellvoltage3);
    _mav_put_uint16_t(buf, 34, cellvoltage4);
    _mav_put_uint16_t(buf, 36, cellvoltage5);
    _mav_put_uint16_t(buf, 38, cellvoltage6);
    _mav_put_uint8_t(buf, 40, SoC);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, buf, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#else
    mavlink_sens_batmon_t *packet = (mavlink_sens_batmon_t *)msgbuf;
    packet->batmon_timestamp = batmon_timestamp;
    packet->temperature = temperature;
    packet->safetystatus = safetystatus;
    packet->operationstatus = operationstatus;
    packet->voltage = voltage;
    packet->current = current;
    packet->batterystatus = batterystatus;
    packet->serialnumber = serialnumber;
    packet->cellvoltage1 = cellvoltage1;
    packet->cellvoltage2 = cellvoltage2;
    packet->cellvoltage3 = cellvoltage3;
    packet->cellvoltage4 = cellvoltage4;
    packet->cellvoltage5 = cellvoltage5;
    packet->cellvoltage6 = cellvoltage6;
    packet->SoC = SoC;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SENS_BATMON, (const char *)packet, MAVLINK_MSG_ID_SENS_BATMON_MIN_LEN, MAVLINK_MSG_ID_SENS_BATMON_LEN, MAVLINK_MSG_ID_SENS_BATMON_CRC);
#endif
}
#endif

#endif

// MESSAGE SENS_BATMON UNPACKING


/**
 * @brief Get field batmon_timestamp from sens_batmon message
 *
 * @return [us] Time since system start
 */
static inline uint64_t mavlink_msg_sens_batmon_get_batmon_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field temperature from sens_batmon message
 *
 * @return [degC] Battery pack temperature
 */
static inline float mavlink_msg_sens_batmon_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field voltage from sens_batmon message
 *
 * @return [mV] Battery pack voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field current from sens_batmon message
 *
 * @return [mA] Battery pack current
 */
static inline int16_t mavlink_msg_sens_batmon_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field SoC from sens_batmon message
 *
 * @return  Battery pack state-of-charge
 */
static inline uint8_t mavlink_msg_sens_batmon_get_SoC(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field batterystatus from sens_batmon message
 *
 * @return  Battery monitor status report bits in Hex
 */
static inline uint16_t mavlink_msg_sens_batmon_get_batterystatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  24);
}

/**
 * @brief Get field serialnumber from sens_batmon message
 *
 * @return  Battery monitor serial number in Hex
 */
static inline uint16_t mavlink_msg_sens_batmon_get_serialnumber(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  26);
}

/**
 * @brief Get field safetystatus from sens_batmon message
 *
 * @return  Battery monitor safetystatus report bits in Hex
 */
static inline uint32_t mavlink_msg_sens_batmon_get_safetystatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field operationstatus from sens_batmon message
 *
 * @return  Battery monitor operation status report bits in Hex
 */
static inline uint32_t mavlink_msg_sens_batmon_get_operationstatus(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Get field cellvoltage1 from sens_batmon message
 *
 * @return [mV] Battery pack cell 1 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  28);
}

/**
 * @brief Get field cellvoltage2 from sens_batmon message
 *
 * @return [mV] Battery pack cell 2 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  30);
}

/**
 * @brief Get field cellvoltage3 from sens_batmon message
 *
 * @return [mV] Battery pack cell 3 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field cellvoltage4 from sens_batmon message
 *
 * @return [mV] Battery pack cell 4 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  34);
}

/**
 * @brief Get field cellvoltage5 from sens_batmon message
 *
 * @return [mV] Battery pack cell 5 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage5(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field cellvoltage6 from sens_batmon message
 *
 * @return [mV] Battery pack cell 6 voltage
 */
static inline uint16_t mavlink_msg_sens_batmon_get_cellvoltage6(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  38);
}

/**
 * @brief Decode a sens_batmon message into a struct
 *
 * @param msg The message to decode
 * @param sens_batmon C-struct to decode the message contents into
 */
static inline void mavlink_msg_sens_batmon_decode(const mavlink_message_t* msg, mavlink_sens_batmon_t* sens_batmon)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sens_batmon->batmon_timestamp = mavlink_msg_sens_batmon_get_batmon_timestamp(msg);
    sens_batmon->temperature = mavlink_msg_sens_batmon_get_temperature(msg);
    sens_batmon->safetystatus = mavlink_msg_sens_batmon_get_safetystatus(msg);
    sens_batmon->operationstatus = mavlink_msg_sens_batmon_get_operationstatus(msg);
    sens_batmon->voltage = mavlink_msg_sens_batmon_get_voltage(msg);
    sens_batmon->current = mavlink_msg_sens_batmon_get_current(msg);
    sens_batmon->batterystatus = mavlink_msg_sens_batmon_get_batterystatus(msg);
    sens_batmon->serialnumber = mavlink_msg_sens_batmon_get_serialnumber(msg);
    sens_batmon->cellvoltage1 = mavlink_msg_sens_batmon_get_cellvoltage1(msg);
    sens_batmon->cellvoltage2 = mavlink_msg_sens_batmon_get_cellvoltage2(msg);
    sens_batmon->cellvoltage3 = mavlink_msg_sens_batmon_get_cellvoltage3(msg);
    sens_batmon->cellvoltage4 = mavlink_msg_sens_batmon_get_cellvoltage4(msg);
    sens_batmon->cellvoltage5 = mavlink_msg_sens_batmon_get_cellvoltage5(msg);
    sens_batmon->cellvoltage6 = mavlink_msg_sens_batmon_get_cellvoltage6(msg);
    sens_batmon->SoC = mavlink_msg_sens_batmon_get_SoC(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SENS_BATMON_LEN? msg->len : MAVLINK_MSG_ID_SENS_BATMON_LEN;
        memset(sens_batmon, 0, MAVLINK_MSG_ID_SENS_BATMON_LEN);
    memcpy(sens_batmon, _MAV_PAYLOAD(msg), len);
#endif
}
