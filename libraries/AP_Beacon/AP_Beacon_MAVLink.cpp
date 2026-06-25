#include "AP_Beacon_config.h"

#if AP_BEACON_MAVLINK_ENABLED

#include "AP_Beacon_MAVLink.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

bool AP_Beacon_MAVLink::healthy()
{
  return (AP_HAL::millis() - last_update_ms) < 1000;
}

// handle mavlink message
void AP_Beacon_MAVLink::handle_msg(const mavlink_message_t &msg)
{
  if (msg.msgid != MAVLINK_MSG_ID_RANGING_BEACON) {
    return;
  }

  mavlink_ranging_beacon_t bcn_range;
  mavlink_msg_ranging_beacon_decode(&msg, &bcn_range);

  if (!handle_range_msg(bcn_range)) {
    return;
  }

  last_update_ms = AP_HAL::millis();
}

bool AP_Beacon_MAVLink::handle_range_msg(const mavlink_ranging_beacon_t& bcn_range)
{
  if (bcn_range.beacon_id >= AP_BEACON_MAX_BEACONS) {
    return false;
  }

  // Discard until the EKF has an origin to translate against.
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return false;
  }

  const Location beacon_loc(
    bcn_range.lat,
    bcn_range.lon,
    (int32_t)bcn_range.alt * 100,
    Location::AltFrame::ABSOLUTE);
  const Vector3f beacon_ned = ekf_origin.get_distance_NED(beacon_loc);

  const float range_m = bcn_range.range / 1000.0f; // mm -> m

  set_beacon_position(bcn_range.beacon_id, beacon_ned);
  set_beacon_distance(bcn_range.beacon_id, range_m);

  return true;
}

#endif // AP_BEACON_MAVLINK_ENABLED
