// MESSAGE SET_CAM_SHUTTER PACKING

#define MAVLINK_MSG_ID_SET_CAM_SHUTTER 100

typedef struct __mavlink_set_cam_shutter_t 
{
	uint8_t cam_no; ///< Camera id
	uint8_t cam_mode; ///< Camera mode: 0 = auto, 1 = manual
	uint8_t trigger_pin; ///< Trigger pin, 0-3 for PtGrey FireFly
	uint16_t interval; ///< Shutter interval, in microseconds
	uint16_t exposure; ///< Exposure time, in microseconds
	float gain; ///< Camera gain

} mavlink_set_cam_shutter_t;



/**
 * @brief Send a set_cam_shutter message
 *
 * @param cam_no Camera id
 * @param cam_mode Camera mode: 0 = auto, 1 = manual
 * @param trigger_pin Trigger pin, 0-3 for PtGrey FireFly
 * @param interval Shutter interval, in microseconds
 * @param exposure Exposure time, in microseconds
 * @param gain Camera gain
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_cam_shutter_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t cam_no, uint8_t cam_mode, uint8_t trigger_pin, uint16_t interval, uint16_t exposure, float gain)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_SET_CAM_SHUTTER;

	i += put_uint8_t_by_index(cam_no, i, msg->payload); //Camera id
	i += put_uint8_t_by_index(cam_mode, i, msg->payload); //Camera mode: 0 = auto, 1 = manual
	i += put_uint8_t_by_index(trigger_pin, i, msg->payload); //Trigger pin, 0-3 for PtGrey FireFly
	i += put_uint16_t_by_index(interval, i, msg->payload); //Shutter interval, in microseconds
	i += put_uint16_t_by_index(exposure, i, msg->payload); //Exposure time, in microseconds
	i += put_float_by_index(gain, i, msg->payload); //Camera gain

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

static inline uint16_t mavlink_msg_set_cam_shutter_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_cam_shutter_t* set_cam_shutter)
{
	return mavlink_msg_set_cam_shutter_pack(system_id, component_id, msg, set_cam_shutter->cam_no, set_cam_shutter->cam_mode, set_cam_shutter->trigger_pin, set_cam_shutter->interval, set_cam_shutter->exposure, set_cam_shutter->gain);
}

#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_cam_shutter_send(mavlink_channel_t chan, uint8_t cam_no, uint8_t cam_mode, uint8_t trigger_pin, uint16_t interval, uint16_t exposure, float gain)
{
	mavlink_message_t msg;
	mavlink_msg_set_cam_shutter_pack(mavlink_system.sysid, mavlink_system.compid, &msg, cam_no, cam_mode, trigger_pin, interval, exposure, gain);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE SET_CAM_SHUTTER UNPACKING

/**
 * @brief Get field cam_no from set_cam_shutter message
 *
 * @return Camera id
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_cam_no(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field cam_mode from set_cam_shutter message
 *
 * @return Camera mode: 0 = auto, 1 = manual
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_cam_mode(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field trigger_pin from set_cam_shutter message
 *
 * @return Trigger pin, 0-3 for PtGrey FireFly
 */
static inline uint8_t mavlink_msg_set_cam_shutter_get_trigger_pin(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
}

/**
 * @brief Get field interval from set_cam_shutter message
 *
 * @return Shutter interval, in microseconds
 */
static inline uint16_t mavlink_msg_set_cam_shutter_get_interval(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field exposure from set_cam_shutter message
 *
 * @return Exposure time, in microseconds
 */
static inline uint16_t mavlink_msg_set_cam_shutter_get_exposure(const mavlink_message_t* msg)
{
	generic_16bit r;
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[0];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t))[1];
	return (uint16_t)r.s;
}

/**
 * @brief Get field gain from set_cam_shutter message
 *
 * @return Camera gain
 */
static inline float mavlink_msg_set_cam_shutter_get_gain(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(uint16_t)+sizeof(uint16_t))[3];
	return (float)r.f;
}

static inline void mavlink_msg_set_cam_shutter_decode(const mavlink_message_t* msg, mavlink_set_cam_shutter_t* set_cam_shutter)
{
	set_cam_shutter->cam_no = mavlink_msg_set_cam_shutter_get_cam_no(msg);
	set_cam_shutter->cam_mode = mavlink_msg_set_cam_shutter_get_cam_mode(msg);
	set_cam_shutter->trigger_pin = mavlink_msg_set_cam_shutter_get_trigger_pin(msg);
	set_cam_shutter->interval = mavlink_msg_set_cam_shutter_get_interval(msg);
	set_cam_shutter->exposure = mavlink_msg_set_cam_shutter_get_exposure(msg);
	set_cam_shutter->gain = mavlink_msg_set_cam_shutter_get_gain(msg);
}
