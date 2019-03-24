#include "GCS_Tracker.h"
#include "Tracker.h"

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

// update sensors and subsystems present, enabled and healthy flags for reporting to GCS
void GCS_Tracker::update_vehicle_sensor_status_flags()
{
    // default sensors present
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    // first what sensors/controllers we have
    if (AP::compass().enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    const AP_GPS &gps = AP::gps();
    if (gps.status() > AP_GPS::NO_GPS) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    AP_AHRS &ahrs = AP::ahrs();

    const Compass &compass = AP::compass();
    if (AP::compass().enabled() && compass.healthy(0) && ahrs.use_compass()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }
    if (gps.is_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
}

// avoid building/linking Devo:
void AP_DEVO_Telem::init() {};
