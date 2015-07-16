#ifndef __AP_GIMBAL_PARAMETERS__
#define __AP_GIMBAL_PARAMETERS__
#include <AP_Math.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>

enum gmb_param_state_t {
    GMB_PARAMSTATE_NOT_YET_READ=0, // parameter has yet to be initialized
    GMB_PARAMSTATE_FETCH_AGAIN=1, // parameter is being fetched
    GMB_PARAMSTATE_ATTEMPTING_TO_SET=2, // parameter is being set
    GMB_PARAMSTATE_CONSISTENT=3, // parameter is consistent
    GMB_PARAMSTATE_NONEXISTANT=4 // parameter does not seem to exist
};

enum gmb_param_t {
    GMB_PARAM_GMB_OFF_ACC_X=0,
    GMB_PARAM_GMB_OFF_ACC_Y,
    GMB_PARAM_GMB_OFF_ACC_Z,
    GMB_PARAM_GMB_GN_ACC_X,
    GMB_PARAM_GMB_GN_ACC_Y,
    GMB_PARAM_GMB_GN_ACC_Z,
    GMB_PARAM_GMB_OFF_GYRO_X,
    GMB_PARAM_GMB_OFF_GYRO_Y,
    GMB_PARAM_GMB_OFF_GYRO_Z,
    GMB_PARAM_GMB_OFF_JNT_X,
    GMB_PARAM_GMB_OFF_JNT_Y,
    GMB_PARAM_GMB_OFF_JNT_Z,
    GMB_PARAM_GMB_K_RATE,
    GMB_PARAM_GMB_POS_HOLD,
    GMB_PARAM_GMB_MAX_TORQUE,
    GMB_PARAM_GMB_SYSID,
    MAVLINK_GIMBAL_NUM_TRACKED_PARAMS
};

class AP_Gimbal_Parameters
{
public:
    AP_Gimbal_Parameters();

    bool initialized();
    bool received_all();
    void fetch_params();

    void get_param(gmb_param_t param, float& value, float def_val = 0.0f);
    void set_param(mavlink_channel_t chan, gmb_param_t param, float value);

    void update(mavlink_channel_t chan);
    void handle_param_value(DataFlash_Class *dataflash, mavlink_message_t *msg);

    Vector3f get_accel_bias();
    Vector3f get_accel_gain();
    Vector3f get_gyro_bias();
    Vector3f get_joint_bias();
    float get_K_rate();

private:
    static const char* get_param_name(gmb_param_t param);

    static const uint32_t _retry_period;
    static const uint8_t _max_fetch_attempts;
    float _values[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];
    gmb_param_state_t _states[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];

    uint8_t _fetch_attempts[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];
    bool _param_seen[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];

    uint32_t _last_request_ms;
};


#endif
