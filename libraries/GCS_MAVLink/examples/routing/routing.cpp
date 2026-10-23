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

static MAVLink_routing routing;
static mavlink_status_t status;

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

    mavlink_msg_heartbeat_encode_status(3, 1, &status, &msg, &heartbeat);

    GCS_MAVLINK *dummy_link = gcs().chan(0);

    if (!routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("heartbeat should be processed locally\n");
        err_count++;
    }

    // incoming non-targetted message
    mavlink_attitude_t attitude = {0};
    mavlink_msg_attitude_encode_status(3, 1, &status, &msg, &attitude);
    if (!routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("attitude should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for someone else
    mavlink_param_set_t param_set = {0};
    param_set.target_system = mavlink_system.sysid+1;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode_status(3, 1, &status, &msg, &param_set);
    if (routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("param set 1 should not be processed locally\n");
        err_count++;
    }

    // incoming targeted message for us
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid;
    mavlink_msg_param_set_encode_status(3, 1, &status, &msg, &param_set);
    if (!routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("param set 2 should be processed locally\n");
        err_count++;
    }

    // incoming targeted message for our system, but other compid
    // should be processed locally
    param_set.target_system = mavlink_system.sysid;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode_status(3, 1, &status, &msg, &param_set);
    if (!routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("param set 3 should be processed locally\n");
        err_count++;
    }

    // incoming broadcast message should be processed locally
    param_set.target_system = 0;
    param_set.target_component = mavlink_system.compid+1;
    mavlink_msg_param_set_encode_status(3, 1, &status, &msg, &param_set);
    if (!routing.check_and_forward(uint8_t(MAVLINK_FRAMING_OK), *dummy_link, msg)) {
        hal.console->printf("param set 4 should be processed locally\n");
        err_count++;
    }

    // Target-bearing messages use a wide header only for IDs above 255.
    const uint32_t message_ids[] = {
        MAVLINK_MSG_ID_PARAM_SET, MAVLINK_MSG_ID_COMMAND_LONG, MAVLINK_MSG_ID_PING,
    };
    const uint32_t own_ids[] = {1, 100000, 0xFFFFFFFF};
    const uint32_t saved_sysid = mavlink_system.sysid;
    for (const uint32_t own_id : own_ids) {
        mavlink_system.sysid = own_id;
        const uint32_t targets[] = {0, own_id, own_id ^ 0x100, own_id ^ 1};
        for (const uint32_t msgid : message_ids) {
            const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
            for (const uint32_t target : targets) {
                msg = {};
                msg.msgid = msgid;
                _MAV_PAYLOAD_NON_CONST(&msg)[entry->target_system_ofs] = mavlink_msg_target_field(target);
                _MAV_PAYLOAD_NON_CONST(&msg)[entry->target_component_ofs] = mavlink_system.compid;
                mavlink_finalize_message_buffer_target(
                    &msg, 70000, 7, &status, entry->min_msg_len,
                    entry->max_msg_len, entry->crc_extra, target);
                const bool expected = target == 0 || target == own_id;
                if (routing.check_and_forward(MAVLINK_FRAMING_OK, *dummy_link, msg) != expected) {
                    hal.console->printf("header target routing failed: msg=%u own=%u target=%u\n",
                                        unsigned(msgid), unsigned(own_id), unsigned(target));
                    err_count++;
                }
                if (GCS_MAVLINK::packet_overhead_chan(dummy_link->get_chan()) <
                    mavlink_msg_get_send_buffer_length(&msg) - msg.len) {
                    hal.console->printf("extended header space underestimated\n");
                    err_count++;
                }
            }
        }
    }
    mavlink_system.sysid = saved_sysid;

    if (err_count == 0) {
        hal.console->printf("All OK\n");
    }
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
