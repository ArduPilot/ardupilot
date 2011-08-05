// MESSAGE UALBERTA_SYS_STATUS PACKING

#define MAVLINK_MSG_ID_UALBERTA_SYS_STATUS 222

typedef struct __mavlink_ualberta_sys_status_t 
{
	uint8_t mode; ///< System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	uint8_t nav_mode; ///< Navigation mode, see UALBERTA_NAV_MODE ENUM
	uint8_t pilot; ///< Pilot mode, see UALBERTA_PILOT_MODE

} mavlink_ualberta_sys_status_t;



/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;

	i += put_uint8_t_by_index(mode, i, msg->payload); // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	i += put_uint8_t_by_index(nav_mode, i, msg->payload); // Navigation mode, see UALBERTA_NAV_MODE ENUM
	i += put_uint8_t_by_index(pilot, i, msg->payload); // Pilot mode, see UALBERTA_PILOT_MODE

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a ualberta_sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_UALBERTA_SYS_STATUS;

	i += put_uint8_t_by_index(mode, i, msg->payload); // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	i += put_uint8_t_by_index(nav_mode, i, msg->payload); // Navigation mode, see UALBERTA_NAV_MODE ENUM
	i += put_uint8_t_by_index(pilot, i, msg->payload); // Pilot mode, see UALBERTA_PILOT_MODE

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a ualberta_sys_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ualberta_sys_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ualberta_sys_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
	return mavlink_msg_ualberta_sys_status_pack(system_id, component_id, msg, ualberta_sys_status->mode, ualberta_sys_status->nav_mode, ualberta_sys_status->pilot);
}

/**
 * @brief Send a ualberta_sys_status message
 * @param chan MAVLink channel to send the message
 *
 * @param mode System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 * @param nav_mode Navigation mode, see UALBERTA_NAV_MODE ENUM
 * @param pilot Pilot mode, see UALBERTA_PILOT_MODE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ualberta_sys_status_send(mavlink_channel_t chan, uint8_t mode, uint8_t nav_mode, uint8_t pilot)
{
	mavlink_message_t msg;
	mavlink_msg_ualberta_sys_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, mode, nav_mode, pilot);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE UALBERTA_SYS_STATUS UNPACKING

/**
 * @brief Get field mode from ualberta_sys_status message
 *
 * @return System mode, see UALBERTA_AUTOPILOT_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field nav_mode from ualberta_sys_status message
 *
 * @return Navigation mode, see UALBERTA_NAV_MODE ENUM
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_nav_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field pilot from ualberta_sys_status message
 *
 * @return Pilot mode, see UALBERTA_PILOT_MODE
 */
static inline uint8_t mavlink_msg_ualberta_sys_status_get_pilot(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Decode a ualberta_sys_status message into a struct
 *
 * @param msg The message to decode
 * @param ualberta_sys_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_ualberta_sys_status_decode(const mavlink_message_t* msg, mavlink_ualberta_sys_status_t* ualberta_sys_status)
{
	ualberta_sys_status->mode = mavlink_msg_ualberta_sys_status_get_mode(msg);
	ualberta_sys_status->nav_mode = mavlink_msg_ualberta_sys_status_get_nav_mode(msg);
	ualberta_sys_status->pilot = mavlink_msg_ualberta_sys_status_get_pilot(msg);
}
