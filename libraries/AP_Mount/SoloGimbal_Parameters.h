#ifndef __SOLOGIMBAL_PARAMETERS__
#define __SOLOGIMBAL_PARAMETERS__
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <DataFlash/DataFlash.h>

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
    GMB_PARAM_GMB_SND_TORQUE,
    GMB_PARAM_GMB_SYSID,
    GMB_PARAM_GMB_FLASH,
    MAVLINK_GIMBAL_NUM_TRACKED_PARAMS
};

enum gmb_flashing_step_t {
    GMB_PARAM_NOT_FLASHING=0,
    GMB_PARAM_FLASHING_WAITING_FOR_SET,
    GMB_PARAM_FLASHING_WAITING_FOR_ACK
};

class SoloGimbal_Parameters
{
public:
    SoloGimbal_Parameters();
    void reset();

    bool initialized();
    bool received_all();
    void fetch_params();

    void get_param(gmb_param_t param, float& value, float def_val = 0.0f);
    void set_param(gmb_param_t param, float value);

    void update();
    void handle_param_value(DataFlash_Class *dataflash, mavlink_message_t *msg);

    Vector3f get_accel_bias();
    Vector3f get_accel_gain();
    void set_accel_bias(const Vector3f& bias);
    void set_accel_gain(const Vector3f& gain);
    Vector3f get_gyro_bias();
    void set_gyro_bias(const Vector3f& bias);
    Vector3f get_joint_bias();

    float get_K_rate();
    void flash();
    bool flashing();

    void set_channel(mavlink_channel_t chan) { _chan = chan; }

private:
    static const char* get_param_name(gmb_param_t param);

    static const uint32_t _retry_period;
    static const uint8_t _max_fetch_attempts;

    struct {
        float value;
        gmb_param_state_t state;
        uint8_t fetch_attempts;
        bool seen;
    } _params[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];

    uint32_t _last_request_ms;
    gmb_flashing_step_t _flashing_step;

    mavlink_channel_t _chan;
};


#endif // __SOLOGIMBAL_PARAMETERS__
