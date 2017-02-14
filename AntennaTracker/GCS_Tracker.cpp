#include "GCS_Tracker.h"
#include "Tracker.h"

bool GCS_Tracker::cli_enabled() const
{
    return false;
}

AP_HAL::BetterStream* GCS_Tracker::cliSerial() {
    return nullptr;
}

static void mavlink_snoop_static(const mavlink_message_t* msg)
{
    tracker.mavlink_snoop(msg);
}

void GCS_Tracker::setup_uarts(AP_SerialManager &serial_manager)
{
    GCS::setup_uarts(serial_manager);

    for (uint8_t i = 1; i < num_gcs(); i++) {
        gcs().chan(i).set_snoop(mavlink_snoop_static);
    }
}

void GCS_Tracker::request_datastream_position(const uint8_t sysid, const uint8_t compid)
{
    for (uint8_t i=0; i < num_gcs(); i++) {
        if (gcs().chan(i).initialised) {
            // request position
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    sysid,
                    compid,
                    MAV_DATA_STREAM_POSITION,
                    tracker.g.mavlink_update_rate,
                    1); // start streaming
            }
        }
    }
}

void GCS_Tracker::request_datastream_airpressure(const uint8_t sysid, const uint8_t compid)
{
    for (uint8_t i=0; i < num_gcs(); i++) {
        if (gcs().chan(i).initialised) {
            // request air pressure
            if (HAVE_PAYLOAD_SPACE((mavlink_channel_t)i, DATA_STREAM)) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    sysid,
                    compid,
                    MAV_DATA_STREAM_RAW_SENSORS,
                    tracker.g.mavlink_update_rate,
                    1); // start streaming
            }
        }
    }
}
