//
// Simple test for the GCS_MAVLink decode performance
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

void setup(void)
{
    hal.console->printf("routing test startup...");
    gcs().init();
    gcs().setup_console();
}

uint32_t get_rate(bool with_objects);
uint32_t get_rate(bool with_objects)
{
    mavlink_message_t msg;

    // heartbeat
    mavlink_heartbeat_t heartbeat = {};
    mavlink_msg_heartbeat_encode(3, 1, &msg, &heartbeat);
    uint8_t heartbeat_buf[128];
    const uint16_t heartbeat_buf_len = mavlink_msg_to_send_buffer(heartbeat_buf, &msg);

    // attitude
    mavlink_attitude_t attitude = {};
    mavlink_msg_attitude_encode(3, 1, &msg, &attitude);
    uint8_t attitude_buf[128];
    const uint16_t attitude_buf_len = mavlink_msg_to_send_buffer(attitude_buf, &msg);

    const uint64_t tstart = AP_HAL::micros64();

    mavlink_status_t status;
    mavlink_message_t buffer;

    const uint32_t iterations = 1 << 16;
    uint32_t heartbeats_received = 0;
    uint32_t attitudes_received = 0;
    for (uint32_t iter=0; iter < iterations; iter++) {
        for (uint16_t i=0; i<heartbeat_buf_len; i++) {
            if (with_objects) {
                if (mavlink_parse_char(1, heartbeat_buf[i], &buffer, &status)) {
                    heartbeats_received++;
                }
            } else {
                if (mavlink_parse_char(1, heartbeat_buf[i], NULL, NULL)) {
                    heartbeats_received++;
                }
            }
        }
        for (uint16_t i=0; i<attitude_buf_len; i++) {
            if (with_objects) {
                if (mavlink_parse_char(1, attitude_buf[i], &buffer, &status)) {
                    attitudes_received++;
                }
            } else {
                if (mavlink_parse_char(1, attitude_buf[i], NULL, NULL)) {
                    attitudes_received++;
                }
            }
        }
    }

    const uint64_t tdelta = AP_HAL::micros64() - tstart;

    if (heartbeats_received != iterations) {
        hal.console->printf("Did not get correct number of heartbeats: want=%u got=%u", (unsigned)iterations, (unsigned)heartbeats_received);
    }
    if (attitudes_received != iterations) {
        hal.console->printf("Did not get correct number of attitudes: want=%u got=%u", (unsigned)iterations, (unsigned)attitudes_received);
    }

    const uint32_t bytes_decoded = iterations * (heartbeat_buf_len + attitude_buf_len);
    hal.console->printf("dec=%u t=%" PRIu64 "\n", (unsigned)bytes_decoded, tdelta);
    const uint64_t x = bytes_decoded * 1000000LL;
    return x/tdelta;
}

void loop(void)
{
    hal.console->printf("Decode rate    (with return-objects): %u bytes/s\n", (unsigned)get_rate(true));
    hal.console->printf("Decode rate (without return-objects): %u bytes/s\n", (unsigned)get_rate(false));

    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
