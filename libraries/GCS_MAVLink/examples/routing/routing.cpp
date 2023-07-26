//
// Simple test for the GCS_MAVLink routing 
//

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_SerialManager/AP_SerialManager.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_SerialManager _serialmanager;
GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

static MAVLink_routing routing;

void setup(void)
{
    hal.console->printf("routing test startup...");
    gcs().init();
    gcs().setup_console();
}

void loop(void)
{
    uint16_t err_count = 0;

    // incoming heartbeat
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat = {0};

    mavlink_msg_heartbeat_encode(3, 1, &msg, &heartbeat);

    GCS_MAVLINK *dummy_link = gcs().chan(0);

    if (!routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("heartbeat should be processed locally\n");
        err_count++;
    }

    // incoming non-targetted message
    mavlink_attitude_t attitude = {0};
    mavlink_msg_attitude_encode(3, 1, &msg, &attitude);
    if (!routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("attitude should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for someone else
    mavlink_param_set_t param_set = {0};
    param_set.target_system = mavlink_system.sysid+1;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("param set 1 should not be processed locally\n");
        err_count++;
    }

    // incoming targeted message for us
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("param set 2 should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for our system, but other compid
    // should be processed locally
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("param set 3 should be processed locally\n");
        err_count++;
    }

    // incoming broadcast message should be processed locally
    param_set.target_system = 0;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(*dummy_link, msg)) {
        hal.console->printf("param set 4 should be processed locally\n");
        err_count++;
    }

    if (err_count == 0) {
        hal.console->printf("All OK\n");
    }
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
