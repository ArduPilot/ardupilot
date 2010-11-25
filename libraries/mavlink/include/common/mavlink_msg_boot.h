// MESSAGE BOOT PACKING

#define MAVLINK_MSG_ID_BOOT 1

typedef struct __mavlink_boot_t 
{
	uint32_t version; ///< The onboard software version

} mavlink_boot_t;



/**
 * @brief Send a boot message
 *
 * @param version The onboard software version
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_boot_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t version)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_BOOT;

	i += put_uint32_t_by_index(version, i, msg->payload); //The onboard software version

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_boot_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_boot_t* boot)
{
	return mavlink_msg_boot_pack(system_id, component_id, msg, boot->version);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_boot_send(mavlink_channel_t chan, uint32_t version)
{
	mavlink_message_t msg;
	mavlink_msg_boot_pack(mavlink_system.sysid, mavlink_system.compid, &msg, version);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE BOOT UNPACKING

/**
 * @brief Get field version from boot message
 *
 * @return The onboard software version
 */
static inline uint32_t mavlink_msg_boot_get_version(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload)[0];
	r.b[2] = (msg->payload)[1];
	r.b[1] = (msg->payload)[2];
	r.b[0] = (msg->payload)[3];
	return (uint32_t)r.i;
}

static inline void mavlink_msg_boot_decode(const mavlink_message_t* msg, mavlink_boot_t* boot)
{
	boot->version = mavlink_msg_boot_get_version(msg);
}
