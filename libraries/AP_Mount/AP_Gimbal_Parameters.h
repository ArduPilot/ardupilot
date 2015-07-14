#ifndef __AP_GIMBAL_PARAMETERS__
#define __AP_GIMBAL_PARAMETERS__
#include <AP_Math.h>
#include <AP_Common.h>
#include <GCS_MAVLink.h>
#include <DataFlash.h>

#define MAVLINK_GIMBAL_NUM_TRACKED_PARAMS 15

enum param_state_t {
    GMB_PARAM_NOT_YET_READ=0, // parameter has yet to be initialized
    GMB_PARAM_FETCH_AGAIN=1, // parameter is being fetched
    GMB_PARAM_ATTEMPTING_TO_SET=2, // parameter is being set
    GMB_PARAM_CONSISTENT=3, // parameter is consistent
    GMB_PARAM_NONEXISTANT=4 // parameter does not seem to exist
};

class AP_Gimbal_Parameters
{
public:
    AP_Gimbal_Parameters();

    bool initialized();
    bool received_all();
    void fetch_params();

    void get_param(char const* name, float& value, float def_val = 0.0f);
    void set_param(mavlink_channel_t chan, char const* name, float value);

    void update(mavlink_channel_t chan);
    void handle_param_value(DataFlash_Class *dataflash, mavlink_message_t *msg);

    Vector3f get_accel_bias();
    Vector3f get_accel_gain();
    Vector3f get_gyro_bias();
    Vector3f get_joint_bias();
    float get_K_rate();

private:
    static const char _names[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS][16];

    static const uint32_t _retry_period;
    static const uint8_t _max_fetch_attempts;
    float _values[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];
    param_state_t _states[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];

    uint8_t _fetch_attempts[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];
    bool _param_seen[MAVLINK_GIMBAL_NUM_TRACKED_PARAMS];

    uint32_t _last_request_ms;
};


#endif
