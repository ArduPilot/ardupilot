//
// Simple test for the GCS_MAVLink routing 
//

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

const AP_FWVersion AP_FWVersion::fwver
{
    major: 3,
    minor: 1,
    patch: 4,
    fw_type: FIRMWARE_VERSION_TYPE_DEV,
    fw_string: "routing example"
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] {};

class GCS_MAVLINK_routing : public GCS_MAVLINK
{

public:

protected:

    uint32_t telem_delay() const override { return 0; }
    uint8_t sysid_my_gcs() const override { return 1; }
    bool set_mode(uint8_t mode) override { return false; };

    // dummy information:
    MAV_MODE base_mode() const override { return (MAV_MODE)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; }
    MAV_STATE system_status() const override { return MAV_STATE_CALIBRATING; }
    void send_nav_controller_output() const override {};
    void send_pid_tuning() override {};

    bool set_home_to_current_location(bool lock) override { return false; }
    bool set_home(const Location& loc, bool lock) override { return false; }

private:

    void handleMessage(mavlink_message_t * msg) { }
    bool handle_guided_request(AP_Mission::Mission_Command &cmd) override { return false ; }
    void handle_change_alt_request(AP_Mission::Mission_Command &cmd) override { }
    bool try_send_message(enum ap_message id) override { return false; }

};


static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK_routing gcs_link[MAVLINK_COMM_NUM_BUFFERS];

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};

static MAVLink_routing routing;

void setup(void)
{
    hal.console->printf("routing test startup...");
    gcs_link[0].init(hal.uartA, MAVLINK_COMM_0);
}

void loop(void)
{
    uint16_t err_count = 0;

    // incoming heartbeat
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat = {0};

    mavlink_msg_heartbeat_encode(3, 1, &msg, &heartbeat);

    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("heartbeat should be processed locally\n");
        err_count++;
    }

    // incoming non-targetted message
    mavlink_attitude_t attitude = {0};
    mavlink_msg_attitude_encode(3, 1, &msg, &attitude);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("attitude should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for someone else
    mavlink_param_set_t param_set = {0};
    param_set.target_system = mavlink_system.sysid+1;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 1 should not be processed locally\n");
        err_count++;
    }

    // incoming targeted message for us
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 2 should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for our system, but other compid
    // should be processed locally
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 3 should be processed locally\n");
        err_count++;
    }

    // incoming broadcast message should be processed locally
    param_set.target_system = 0;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode(3, 1, &msg, &param_set);
    if (!routing.check_and_forward(MAVLINK_COMM_0, &msg)) {
        hal.console->printf("param set 4 should be processed locally\n");
        err_count++;
    }

    if (err_count == 0) {
        hal.console->printf("All OK\n");
    }
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
