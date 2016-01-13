#include "GCS_MAVLink/GCS.h"

class GCS_Backend_Rover : public GCS_MAVLINK {

public:

    bool stream_trigger(enum streams stream_num) override;
    void data_stream_send(void) override;
    void handle_guided_request(AP_Mission::Mission_Command &cmd) override;
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override;
    void handleMessage(mavlink_message_t* msg) override;

protected:

    AP_GPS &_gps() const override;

#if AP_AHRS_NAVEKF_AVAILABLE
    AP_AHRS_NavEKF &_ahrs() const override;
#else
    AP_AHRS &_ahrs() const override;
#endif

    bool should_try_send_message(enum ap_message id) override;

    bool send_AHRS() override;
    bool send_AHRS2() override;
    bool send_ATTITUDE() override;
    bool send_BATTERY2() override;
    bool send_CAMERA_FEEDBACK() override;
    bool send_EKF_STATUS_REPORT() override;
    bool send_GLOBAL_POSITION_INT() override;
    bool send_HEARTBEAT() override;
    bool send_HWSTATUS() override;
    bool send_LOCAL_POSITION_NED() override;
    bool send_MAG_CAL_PROGRESS() override;
    bool send_MAG_CAL_REPORT() override;
    bool send_MISSION_CURRENT() override;
    bool send_MISSION_ITEM_REACHED() override;
    bool send_MOUNT_STATUS() override;
    bool send_NAV_CONTROLLER_OUTPUT() override;
    bool send_PID_TUNING() override;
    bool send_RANGEFINDER() override;
    bool send_RAW_IMU() override;
    bool send_RC_CHANNELS_RAW() override;
    bool send_RC_CHANNELS_SCALED() override;
    bool send_SENSOR_OFFSETS() override;
    bool send_SERVO_OUTPUT_RAW() override;
    bool send_SIMSTATE() override;
    bool send_STATUSTEXT() override;
    bool send_SYS_STATUS() override;
    bool send_SYSTEM_TIME() override;
    bool send_VFR_HUD() override;

private:

};
