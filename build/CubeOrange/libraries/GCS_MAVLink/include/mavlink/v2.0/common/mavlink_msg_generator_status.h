#pragma once
// MESSAGE GENERATOR_STATUS PACKING

#define MAVLINK_MSG_ID_GENERATOR_STATUS 373


typedef struct __mavlink_generator_status_t {
 uint64_t status; /*<  Status flags.*/
 float battery_current; /*< [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.*/
 float load_current; /*< [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided*/
 float power_generated; /*< [W] The power being generated. NaN: field not provided*/
 float bus_voltage; /*< [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.*/
 float bat_current_setpoint; /*< [A] The target battery current. Positive for out. Negative for in. NaN: field not provided*/
 uint32_t runtime; /*< [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.*/
 int32_t time_until_maintenance; /*< [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.*/
 uint16_t generator_speed; /*< [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.*/
 int16_t rectifier_temperature; /*< [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.*/
 int16_t generator_temperature; /*< [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.*/
} mavlink_generator_status_t;

#define MAVLINK_MSG_ID_GENERATOR_STATUS_LEN 42
#define MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN 42
#define MAVLINK_MSG_ID_373_LEN 42
#define MAVLINK_MSG_ID_373_MIN_LEN 42

#define MAVLINK_MSG_ID_GENERATOR_STATUS_CRC 117
#define MAVLINK_MSG_ID_373_CRC 117



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GENERATOR_STATUS { \
    373, \
    "GENERATOR_STATUS", \
    11, \
    {  { "status", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_generator_status_t, status) }, \
         { "generator_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_generator_status_t, generator_speed) }, \
         { "battery_current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_generator_status_t, battery_current) }, \
         { "load_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_generator_status_t, load_current) }, \
         { "power_generated", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_generator_status_t, power_generated) }, \
         { "bus_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_generator_status_t, bus_voltage) }, \
         { "rectifier_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_generator_status_t, rectifier_temperature) }, \
         { "bat_current_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_generator_status_t, bat_current_setpoint) }, \
         { "generator_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_generator_status_t, generator_temperature) }, \
         { "runtime", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_generator_status_t, runtime) }, \
         { "time_until_maintenance", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_generator_status_t, time_until_maintenance) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GENERATOR_STATUS { \
    "GENERATOR_STATUS", \
    11, \
    {  { "status", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_generator_status_t, status) }, \
         { "generator_speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_generator_status_t, generator_speed) }, \
         { "battery_current", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_generator_status_t, battery_current) }, \
         { "load_current", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_generator_status_t, load_current) }, \
         { "power_generated", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_generator_status_t, power_generated) }, \
         { "bus_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_generator_status_t, bus_voltage) }, \
         { "rectifier_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_generator_status_t, rectifier_temperature) }, \
         { "bat_current_setpoint", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_generator_status_t, bat_current_setpoint) }, \
         { "generator_temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_generator_status_t, generator_temperature) }, \
         { "runtime", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_generator_status_t, runtime) }, \
         { "time_until_maintenance", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_generator_status_t, time_until_maintenance) }, \
         } \
}
#endif

/**
 * @brief Pack a generator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status  Status flags.
 * @param generator_speed [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.
 * @param battery_current [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
 * @param load_current [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
 * @param power_generated [W] The power being generated. NaN: field not provided
 * @param bus_voltage [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
 * @param rectifier_temperature [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.
 * @param bat_current_setpoint [A] The target battery current. Positive for out. Negative for in. NaN: field not provided
 * @param generator_temperature [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
 * @param runtime [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
 * @param time_until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_generator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GENERATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, status);
    _mav_put_float(buf, 8, battery_current);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power_generated);
    _mav_put_float(buf, 20, bus_voltage);
    _mav_put_float(buf, 24, bat_current_setpoint);
    _mav_put_uint32_t(buf, 28, runtime);
    _mav_put_int32_t(buf, 32, time_until_maintenance);
    _mav_put_uint16_t(buf, 36, generator_speed);
    _mav_put_int16_t(buf, 38, rectifier_temperature);
    _mav_put_int16_t(buf, 40, generator_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN);
#else
    mavlink_generator_status_t packet;
    packet.status = status;
    packet.battery_current = battery_current;
    packet.load_current = load_current;
    packet.power_generated = power_generated;
    packet.bus_voltage = bus_voltage;
    packet.bat_current_setpoint = bat_current_setpoint;
    packet.runtime = runtime;
    packet.time_until_maintenance = time_until_maintenance;
    packet.generator_speed = generator_speed;
    packet.rectifier_temperature = rectifier_temperature;
    packet.generator_temperature = generator_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GENERATOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
}

/**
 * @brief Pack a generator_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param status  Status flags.
 * @param generator_speed [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.
 * @param battery_current [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
 * @param load_current [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
 * @param power_generated [W] The power being generated. NaN: field not provided
 * @param bus_voltage [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
 * @param rectifier_temperature [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.
 * @param bat_current_setpoint [A] The target battery current. Positive for out. Negative for in. NaN: field not provided
 * @param generator_temperature [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
 * @param runtime [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
 * @param time_until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_generator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t status,uint16_t generator_speed,float battery_current,float load_current,float power_generated,float bus_voltage,int16_t rectifier_temperature,float bat_current_setpoint,int16_t generator_temperature,uint32_t runtime,int32_t time_until_maintenance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GENERATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, status);
    _mav_put_float(buf, 8, battery_current);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power_generated);
    _mav_put_float(buf, 20, bus_voltage);
    _mav_put_float(buf, 24, bat_current_setpoint);
    _mav_put_uint32_t(buf, 28, runtime);
    _mav_put_int32_t(buf, 32, time_until_maintenance);
    _mav_put_uint16_t(buf, 36, generator_speed);
    _mav_put_int16_t(buf, 38, rectifier_temperature);
    _mav_put_int16_t(buf, 40, generator_temperature);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN);
#else
    mavlink_generator_status_t packet;
    packet.status = status;
    packet.battery_current = battery_current;
    packet.load_current = load_current;
    packet.power_generated = power_generated;
    packet.bus_voltage = bus_voltage;
    packet.bat_current_setpoint = bat_current_setpoint;
    packet.runtime = runtime;
    packet.time_until_maintenance = time_until_maintenance;
    packet.generator_speed = generator_speed;
    packet.rectifier_temperature = rectifier_temperature;
    packet.generator_temperature = generator_temperature;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GENERATOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
}

/**
 * @brief Encode a generator_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param generator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_generator_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_generator_status_t* generator_status)
{
    return mavlink_msg_generator_status_pack(system_id, component_id, msg, generator_status->status, generator_status->generator_speed, generator_status->battery_current, generator_status->load_current, generator_status->power_generated, generator_status->bus_voltage, generator_status->rectifier_temperature, generator_status->bat_current_setpoint, generator_status->generator_temperature, generator_status->runtime, generator_status->time_until_maintenance);
}

/**
 * @brief Encode a generator_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param generator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_generator_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_generator_status_t* generator_status)
{
    return mavlink_msg_generator_status_pack_chan(system_id, component_id, chan, msg, generator_status->status, generator_status->generator_speed, generator_status->battery_current, generator_status->load_current, generator_status->power_generated, generator_status->bus_voltage, generator_status->rectifier_temperature, generator_status->bat_current_setpoint, generator_status->generator_temperature, generator_status->runtime, generator_status->time_until_maintenance);
}

/**
 * @brief Send a generator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status  Status flags.
 * @param generator_speed [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.
 * @param battery_current [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
 * @param load_current [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
 * @param power_generated [W] The power being generated. NaN: field not provided
 * @param bus_voltage [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
 * @param rectifier_temperature [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.
 * @param bat_current_setpoint [A] The target battery current. Positive for out. Negative for in. NaN: field not provided
 * @param generator_temperature [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
 * @param runtime [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
 * @param time_until_maintenance [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_generator_status_send(mavlink_channel_t chan, uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GENERATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, status);
    _mav_put_float(buf, 8, battery_current);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power_generated);
    _mav_put_float(buf, 20, bus_voltage);
    _mav_put_float(buf, 24, bat_current_setpoint);
    _mav_put_uint32_t(buf, 28, runtime);
    _mav_put_int32_t(buf, 32, time_until_maintenance);
    _mav_put_uint16_t(buf, 36, generator_speed);
    _mav_put_int16_t(buf, 38, rectifier_temperature);
    _mav_put_int16_t(buf, 40, generator_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GENERATOR_STATUS, buf, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
#else
    mavlink_generator_status_t packet;
    packet.status = status;
    packet.battery_current = battery_current;
    packet.load_current = load_current;
    packet.power_generated = power_generated;
    packet.bus_voltage = bus_voltage;
    packet.bat_current_setpoint = bat_current_setpoint;
    packet.runtime = runtime;
    packet.time_until_maintenance = time_until_maintenance;
    packet.generator_speed = generator_speed;
    packet.rectifier_temperature = rectifier_temperature;
    packet.generator_temperature = generator_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GENERATOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a generator_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_generator_status_send_struct(mavlink_channel_t chan, const mavlink_generator_status_t* generator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_generator_status_send(chan, generator_status->status, generator_status->generator_speed, generator_status->battery_current, generator_status->load_current, generator_status->power_generated, generator_status->bus_voltage, generator_status->rectifier_temperature, generator_status->bat_current_setpoint, generator_status->generator_temperature, generator_status->runtime, generator_status->time_until_maintenance);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GENERATOR_STATUS, (const char *)generator_status, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_GENERATOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_generator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t status, uint16_t generator_speed, float battery_current, float load_current, float power_generated, float bus_voltage, int16_t rectifier_temperature, float bat_current_setpoint, int16_t generator_temperature, uint32_t runtime, int32_t time_until_maintenance)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, status);
    _mav_put_float(buf, 8, battery_current);
    _mav_put_float(buf, 12, load_current);
    _mav_put_float(buf, 16, power_generated);
    _mav_put_float(buf, 20, bus_voltage);
    _mav_put_float(buf, 24, bat_current_setpoint);
    _mav_put_uint32_t(buf, 28, runtime);
    _mav_put_int32_t(buf, 32, time_until_maintenance);
    _mav_put_uint16_t(buf, 36, generator_speed);
    _mav_put_int16_t(buf, 38, rectifier_temperature);
    _mav_put_int16_t(buf, 40, generator_temperature);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GENERATOR_STATUS, buf, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
#else
    mavlink_generator_status_t *packet = (mavlink_generator_status_t *)msgbuf;
    packet->status = status;
    packet->battery_current = battery_current;
    packet->load_current = load_current;
    packet->power_generated = power_generated;
    packet->bus_voltage = bus_voltage;
    packet->bat_current_setpoint = bat_current_setpoint;
    packet->runtime = runtime;
    packet->time_until_maintenance = time_until_maintenance;
    packet->generator_speed = generator_speed;
    packet->rectifier_temperature = rectifier_temperature;
    packet->generator_temperature = generator_temperature;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GENERATOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_GENERATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN, MAVLINK_MSG_ID_GENERATOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE GENERATOR_STATUS UNPACKING


/**
 * @brief Get field status from generator_status message
 *
 * @return  Status flags.
 */
static inline uint64_t mavlink_msg_generator_status_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field generator_speed from generator_status message
 *
 * @return [rpm] Speed of electrical generator or alternator. UINT16_MAX: field not provided.
 */
static inline uint16_t mavlink_msg_generator_status_get_generator_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field battery_current from generator_status message
 *
 * @return [A] Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
 */
static inline float mavlink_msg_generator_status_get_battery_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field load_current from generator_status message
 *
 * @return [A] Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
 */
static inline float mavlink_msg_generator_status_get_load_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field power_generated from generator_status message
 *
 * @return [W] The power being generated. NaN: field not provided
 */
static inline float mavlink_msg_generator_status_get_power_generated(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field bus_voltage from generator_status message
 *
 * @return [V] Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
 */
static inline float mavlink_msg_generator_status_get_bus_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field rectifier_temperature from generator_status message
 *
 * @return [degC] The temperature of the rectifier or power converter. INT16_MAX: field not provided.
 */
static inline int16_t mavlink_msg_generator_status_get_rectifier_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field bat_current_setpoint from generator_status message
 *
 * @return [A] The target battery current. Positive for out. Negative for in. NaN: field not provided
 */
static inline float mavlink_msg_generator_status_get_bat_current_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field generator_temperature from generator_status message
 *
 * @return [degC] The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
 */
static inline int16_t mavlink_msg_generator_status_get_generator_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field runtime from generator_status message
 *
 * @return [s] Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
 */
static inline uint32_t mavlink_msg_generator_status_get_runtime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field time_until_maintenance from generator_status message
 *
 * @return [s] Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.
 */
static inline int32_t mavlink_msg_generator_status_get_time_until_maintenance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Decode a generator_status message into a struct
 *
 * @param msg The message to decode
 * @param generator_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_generator_status_decode(const mavlink_message_t* msg, mavlink_generator_status_t* generator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    generator_status->status = mavlink_msg_generator_status_get_status(msg);
    generator_status->battery_current = mavlink_msg_generator_status_get_battery_current(msg);
    generator_status->load_current = mavlink_msg_generator_status_get_load_current(msg);
    generator_status->power_generated = mavlink_msg_generator_status_get_power_generated(msg);
    generator_status->bus_voltage = mavlink_msg_generator_status_get_bus_voltage(msg);
    generator_status->bat_current_setpoint = mavlink_msg_generator_status_get_bat_current_setpoint(msg);
    generator_status->runtime = mavlink_msg_generator_status_get_runtime(msg);
    generator_status->time_until_maintenance = mavlink_msg_generator_status_get_time_until_maintenance(msg);
    generator_status->generator_speed = mavlink_msg_generator_status_get_generator_speed(msg);
    generator_status->rectifier_temperature = mavlink_msg_generator_status_get_rectifier_temperature(msg);
    generator_status->generator_temperature = mavlink_msg_generator_status_get_generator_temperature(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GENERATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_GENERATOR_STATUS_LEN;
        memset(generator_status, 0, MAVLINK_MSG_ID_GENERATOR_STATUS_LEN);
    memcpy(generator_status, _MAV_PAYLOAD(msg), len);
#endif
}
