#include "GCS_Frontend.h"

#include "Tracker.h"

void GCS_Frontend_Tracker::setup_uarts(AP_SerialManager &serial_manager, void (&mavlink_snoop)(const mavlink_message_t*) )
{
    GCS_Frontend::setup_uarts(serial_manager);

    for (uint8_t i=0; i<num_gcs(); i++) {
        gcs(i).set_snoop(mavlink_snoop);
    }
}

void GCS_Frontend_Tracker::request_data_streams(const uint8_t sysid, const uint8_t compid)
{
    for (uint8_t i=0; i < num_gcs(); i++) {
        if (gcs(i).initialised) {
            // request position
            if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= MAVLINK_MSG_ID_DATA_STREAM_LEN) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    sysid,
                    compid,
                    MAV_DATA_STREAM_POSITION,
                    1,  // 1hz
                    1); // start streaming
            }
            // request air pressure
            if (comm_get_txspace((mavlink_channel_t)i) - MAVLINK_NUM_NON_PAYLOAD_BYTES >= MAVLINK_MSG_ID_DATA_STREAM_LEN) {
                mavlink_msg_request_data_stream_send(
                    (mavlink_channel_t)i,
                    sysid,
                    compid,
                    MAV_DATA_STREAM_RAW_SENSORS,
                    1,  // 1hz
                    1); // start streaming
            }
        }
    }
}

// FIXME: we have no parameter for telem_delay
uint32_t GCS_Frontend_Tracker::telem_delay() const
{
    // return (uint32_t)_g.telem_delay;
    return (uint32_t)0;
}
