#include <AP_Gimbal_Parameters.h>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const static uint32_t retry_period = 3000;

bool AP_Gimbal_Parameters::received_all()
{
	return _mask == MAVLINK_GIMBAL_PARAM_MASK_ALL;
}

// request missing parameters
void AP_Gimbal_Parameters::receive_missing_parameters(mavlink_channel_t chan)
{
    if(received_all()){
    	return;
    }

	static uint32_t last_request = 0;
    uint32_t current_time = hal.scheduler->millis();
    if ((current_time - last_request)>retry_period) {
    	last_request = current_time;
    	mavlink_msg_param_request_list_send(chan,0,MAV_COMP_ID_GIMBAL);
    }
}

void AP_Gimbal_Parameters::handle_param_value(mavlink_message_t *msg)
{
    mavlink_param_value_t packet;
    mavlink_msg_param_value_decode(msg, &packet);
    
    if (!strcmp(packet.param_id, "GMB_OFF_ACC_X")){
        delta_velocity_offsets.x = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X;
    }else if (!strcmp(packet.param_id, "GMB_OFF_ACC_Y")){
        delta_velocity_offsets.y = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y;
    }else if (!strcmp(packet.param_id, "GMB_OFF_ACC_Z")){
        delta_velocity_offsets.z = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z;
    }else if (!strcmp(packet.param_id, "GMB_OFF_GYRO_X")){
        delta_angles_offsets.x = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X;
    }else if (!strcmp(packet.param_id, "GMB_OFF_GYRO_Y")){
        delta_angles_offsets.y = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y;
    }else if (!strcmp(packet.param_id, "GMB_OFF_GYRO_Z")){
        delta_angles_offsets.z = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z;
    }else if (!strcmp(packet.param_id, "GMB_OFF_JNT_X")){
        joint_angles_offsets.x = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X;
    }else if (!strcmp(packet.param_id, "GMB_OFF_JNT_Y")){
        joint_angles_offsets.y = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y;
    }else if (!strcmp(packet.param_id, "GMB_OFF_JNT_Z")){
        joint_angles_offsets.z = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z;
    }else if (!strcmp(packet.param_id, "GMB_K_RATE")){
        K_gimbalRate = packet.param_value;
        _mask|= MAVLINK_GIMBAL_PARAM_GMB_K_RATE;
    }
}