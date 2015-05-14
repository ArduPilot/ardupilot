#include <AP_Math.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>


#define MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X      0x001
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y      0x002
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z      0x004
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X      0x008
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y      0x010
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z      0x020
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X     0x040
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y     0x080
#define MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z     0x0100
#define MAVLINK_GIMBAL_PARAM_GMB_K_RATE         0x0200
#define MAVLINK_GIMBAL_PARAM_MASK_ALL           0x03FF
#define MAVLINK_GIMBAL_PARAM_MASK_NONE          0x0000

class AP_Gimbal_Parameters
{
public:
    Vector3f     delta_angles_offsets;
    Vector3f     delta_velocity_offsets;
    Vector3f     joint_angles_offsets;
    float        K_gimbalRate;
    
    bool received_all();
	void handle_param_value(mavlink_message_t *msg);
	void receive_missing_parameters(mavlink_channel_t chan);

	AP_Gimbal_Parameters(){
		_mask = MAVLINK_GIMBAL_PARAM_MASK_NONE;
	};

private:
	uint16_t     _mask;

};