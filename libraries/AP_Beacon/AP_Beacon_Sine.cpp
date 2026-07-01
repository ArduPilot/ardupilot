#include "AP_Beacon_config.h"

#if AP_BEACON_SINE_ENABLED

#include "AP_Beacon_Sine.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/Location.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Beacon_Sine::AP_Beacon_Sine(AP_Beacon &frontend)
    : AP_Beacon_Backend(frontend)
{
  // Pre-register all beacons so num_beacons is non-zero when the DAL
  // snapshots the count on the first EKF frame.  Real position and distance
  // values are filled in once measurements arrive.
  for (uint8_t i = 0; i < AP_BEACON_MAX_BEACONS; i++) {
    set_beacon_position(i, Vector3f(10.0f, 10.0f, 0.0f));
    set_beacon_distance(i, -1.0f);
  }
}

bool AP_Beacon_Sine::healthy()
{
  return (AP_HAL::millis() - last_update_ms) < 1000;
}

void AP_Beacon_Sine::update(void)
{
  if (!warmup_complete) {
    warmup();
    return;
  }
}

void AP_Beacon_Sine::warmup()
{
  // EKF discards first 100 readings, then uses next 100 to estimate beacon origin.
  // 500 readings at 40Hz (~12.5s) covers the EKF initialization window before flight.
  if (warmup_readings >= 500) {
    warmup_complete = true;
    return;
  }
  
  Location ekf_origin;
  if (!AP::ahrs().get_origin(ekf_origin)) {
    return;
  }

  Location ahrs_loc;
  if (!AP::ahrs().get_location(ahrs_loc)) {
    return;
  }

  if ((AP_HAL::millis() - last_update_ms) < 25) {
    return;
  }

  last_update_ms = AP_HAL::millis();

  static uint8_t fake_bcn_id = 0;

  const Vector3f beacon_ned{10.0f, 0.0f, 0.0f};
  const Vector3f vehicle_ned = ekf_origin.get_distance_NED(ahrs_loc);
  const float dist = (beacon_ned - vehicle_ned).length();
  set_beacon_position(fake_bcn_id, beacon_ned);
  set_beacon_distance(fake_bcn_id, dist);

  warmup_readings++;

  fake_bcn_id++;
  if (fake_bcn_id == AP_BEACON_MAX_BEACONS) {
    fake_bcn_id = 0;
  }
}

// handle mavlink message
void AP_Beacon_Sine::handle_msg(const mavlink_message_t &msg)
{
  if (!warmup_complete) {
    return;
  }

  if (msg.msgid != MAVLINK_MSG_ID_RANGING_BEACON) {
    return;
  }

  mavlink_ranging_beacon_t bcn_range;
  mavlink_msg_ranging_beacon_decode(&msg, &bcn_range);

  // MAV_COMP_ID_USER66 is agreed upon with the Sine ranging firmware
  // as the component ID that identifies RANGING_BEACON messages from
  // Sine beacons, distinguishing them from other beacon systems.
  if (bcn_range.target_component != MAV_COMP_ID_USER66) {
    return;
  }

  if (!handle_range_msg(bcn_range)) {
    return;
  }

  last_update_ms = AP_HAL::millis();
}

bool AP_Beacon_Sine::handle_range_msg(const mavlink_ranging_beacon_t& bcn_range)
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

#endif // AP_BEACON_SINE_ENABLED
