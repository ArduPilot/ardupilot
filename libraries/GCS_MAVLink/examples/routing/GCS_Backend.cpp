#include "GCS_Backend.h"

const AP_Param::GroupInfo GCS_Backend_RoutingExample::var_info[] = {
    AP_GROUPEND
};

bool GCS_Backend_RoutingExample::stream_trigger(enum streams stream_num) { abort(); }
void GCS_Backend_RoutingExample::data_stream_send(void) { abort(); }
void GCS_Backend_RoutingExample::handle_guided_request(AP_Mission::Mission_Command &cmd) { abort(); }
void GCS_Backend_RoutingExample::handle_change_alt_request(AP_Mission::Mission_Command &cmd) { abort(); }
void GCS_Backend_RoutingExample::handleMessage(mavlink_message_t* msg) { abort(); }

AP_GPS &GCS_Backend_RoutingExample::_gps() const { abort(); }

AP_AHRS_NavEKF &GCS_Backend_RoutingExample::_ahrs() const { abort(); }

Compass &GCS_Backend_RoutingExample::_compass() const { abort(); }

AP_Baro &GCS_Backend_RoutingExample::_barometer() const { abort(); }

AP_InertialSensor &GCS_Backend_RoutingExample::_ins() const { abort(); }

bool GCS_Backend_RoutingExample::should_try_send_message(enum ap_message id) { abort(); }

bool GCS_Backend_RoutingExample::send_ATTITUDE() { abort(); }
bool GCS_Backend_RoutingExample::send_GLOBAL_POSITION_INT() { abort(); }
bool GCS_Backend_RoutingExample::send_HEARTBEAT() { abort(); }
bool GCS_Backend_RoutingExample::send_NAV_CONTROLLER_OUTPUT() { abort(); }
bool GCS_Backend_RoutingExample::send_RC_CHANNELS_RAW() { abort(); }
bool GCS_Backend_RoutingExample::send_SERVO_OUTPUT_RAW() { abort(); }
bool GCS_Backend_RoutingExample::send_SIMSTATE() { abort(); }
