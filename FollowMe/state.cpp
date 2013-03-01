
#include "state.h"
#include <AP_HAL.h>
#include <GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;
extern mavlink_channel_t upstream_channel;
extern mavlink_channel_t downstream_channel;

void FMStateMachine::on_upstream_command_long(mavlink_command_long_t* pkt) {
  switch(pkt->command) {
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_MISSION_START:
      /* clear out FM control of vehicle */
      _on_user_override();
    break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      /* i guess do nothing? */
    break;
  }
}

void FMStateMachine::on_upstream_set_mode(mavlink_set_mode_t* pkt) {
  /* mode is set in pkt->custom_mode */
  _vehicle_mode = (int8_t) pkt->custom_mode;
  /* clear out FM control of vehicle */
  _on_user_override();
}

void FMStateMachine::on_downstream_heartbeat(mavlink_heartbeat_t* pkt) {
  /* if mode has changed from last set_mode, the user has triggered a change
   * via RC switch.
   * clear out FM control of vehicle */
  bool pktarmed = ((pkt->base_mode & MAV_MODE_FLAG_SAFETY_ARMED) > 0);
  int8_t pktmode = (int8_t) pkt->custom_mode;
  if ((pktarmed != _vehicle_armed) || (pktmode != _vehicle_mode)) {
    _on_user_override();
  }
  /* update heartbeat millis */
  _last_vehicle_hb_millis = hal.scheduler->millis();
  /* update local state */ 
  _vehicle_armed = pktarmed;
  _vehicle_mode = pktmode;
}

void FMStateMachine::on_downstream_gps_raw_int(mavlink_gps_raw_int_t* pkt) {
  /* Keep track of vehicle's latest lat, lon, altitude */
  _vehicle_lat     = pkt->lat;
  _vehicle_lon     = pkt->lon;
  _vehicle_altitude = pkt->alt;
  _vehicle_gps_fix = pkt->fix_type;
}

void FMStateMachine::on_button_activate() {
  if (_guiding) return;
  /* This action is allowed to swing the state to start guide mode. */
  if (_check_guide_valid()) {
    _set_guide_offset();
    _send_guide();
    _guiding = true;
    _vehicle_mode = MODE_GUIDED;
    hal.console->println_P(PSTR("Button activated, entering guided mode"));
  } else {
    hal.console->println_P(PSTR("Button activated but insufficient conditions "
          "for entering guided mode"));
  }
}

void FMStateMachine::on_button_cancel() {
  if (!_guiding) return;
  _send_loiter();
  _guiding = false;
}

void FMStateMachine::on_loop(GPS* gps) {
  uint32_t now = hal.scheduler->millis();
  if ((_last_run_millis + _loop_period) > now) return;
  _last_run_millis = now;

  if (gps != NULL) {
    _update_local_gps(gps);
  }

  if (_guiding) {
    _send_guide();
  }
}

bool FMStateMachine::_check_guide_valid() {
  uint32_t now = hal.scheduler->millis();

  bool vehicle_gps_valid = (_vehicle_gps_fix == 3);
  bool vehicle_hb_valid = (now - _last_vehicle_hb_millis) < 2000;

  bool vehicle_mode_valid = _vehicle_armed 
                          && ( (_vehicle_mode == MODE_LOITER)
                             ||(_vehicle_mode == MODE_ALT_HOLD)
                             ||(_vehicle_mode == MODE_AUTO)
                             ||(_vehicle_mode == MODE_GUIDED)
                             );
#define DEBUG 1
#if DEBUG
  if (!_local_gps_valid) {
    hal.console->println_P(PSTR("need valid local gps"));
  }
  if (!vehicle_gps_valid) {
    hal.console->println_P(PSTR("need valid vehicle gps"));
  }
  if (!vehicle_hb_valid) {
    hal.console->println_P(PSTR("need valid vehicle hb"));
  }
  if (!vehicle_mode_valid) {
    hal.console->println_P(PSTR("need valid vehicle mode"));
  }
#endif
  return _local_gps_valid
      && vehicle_gps_valid
      && vehicle_hb_valid
      && vehicle_mode_valid;
}

void FMStateMachine::_update_local_gps(GPS* gps) {
  /* Cause an on_fault_cancel if when local gps has transitioned form 
   * valid to invalid. */
  if (_local_gps_valid && !(gps->status() == GPS::GPS_OK)) {
    _on_fault_cancel();
  } 

  _local_gps_valid = (gps->status() == GPS::GPS_OK);
  if (gps->new_data) {
    _local_gps_lat      = gps->latitude;
    _local_gps_lon      = gps->longitude;
    _local_gps_altitude = gps->altitude;
    gps->new_data = false;
  }
}

void FMStateMachine::_set_guide_offset() {
  _offs_lat = 0;
  _offs_lon = 0;
  _offs_altitude = 1200; /* 12m in centimeters */
}

void FMStateMachine::_on_fault_cancel() {
    if (_guiding) { 
        hal.console->println_P(PSTR("FollowMe: Fault Cancel"));
        _send_loiter();
        _guiding = false;
    }
}

void FMStateMachine::_on_user_override() {
    if (_guiding) {
        hal.console->println_P(PSTR("FollowMe: User GCS or RC override"));
        _guiding = false;
    }
}

void FMStateMachine::_send_guide() {
  hal.console->println_P(PSTR("FollowMe: Sending guide waypoint packet"));

  int32_t lat = _local_gps_lat + _offs_lat;
  int32_t lon = _local_gps_lon + _offs_lon;
  // int32_t alt = _local_gps_altitude + _offs_altitude;
  int32_t alt = _offs_altitude; /* assume above ground. (ArduCopter bug.) */

  float x = (float) lat / (float) 1e7; /* lat, lon in deg * 10,000,000 */
  float y = (float) lon / (float) 1e7;
  float z = (float) alt / (float) 100; /* alt in cm */
  
  hal.console->printf_P(
      PSTR("FollowMe: guide x: %f y: %f z: %f\r\n"),
      x, y, z);

  mavlink_msg_mission_item_send(
      upstream_channel, /* mavlink_channel_t chan*/
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      0, /* uint16_t seq: always 0, unknown why. */
      MAV_FRAME_GLOBAL, /* uint8_t frame: arducopter uninterpreted */
      MAV_CMD_NAV_WAYPOINT, /* uint16_t command: arducopter specific */
      2, /* uint8_t current: 2 indicates guided mode waypoint */
      0, /* uint8_t autocontinue: always 0 */
      0, /* float param1 : hold time in seconds */
      5, /* float param2 : acceptance radius in meters */
      0, /* float param3 : pass through waypoint */
      0, /* float param4 : desired yaw angle at waypoint */
      x, /* float x : lat degrees */
      y, /* float y : lon degrees */
      z  /* float z : alt meters */
      );
}

void FMStateMachine::_send_loiter() {
  hal.console->println_P(PSTR("FollowMe: Sending loiter cmd packet"));
  mavlink_msg_command_long_send(
      upstream_channel, /* mavlink_channel_t chan */
      _target_system, /* uint8_t target_system */
      _target_component, /* uint8_t target_component */
      MAV_CMD_NAV_LOITER_UNLIM, /* uint16_t command: arducopter specific */
      0, /* uint8_t confirmation */
      0, /* float param1 */
      0, /* float param2 */
      0, /* float param3 */
      0, /* float param4 */
      0, /* float param5 */
      0, /* float param6 */
      0  /* float param7 */
      );
}
