#include "GCS.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Terrain/AP_Terrain.h>

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::get_item(
    const GCS_MAVLINK &_link,
    const mavlink_message_t &msg,
    const mavlink_mission_request_int_t &packet,
    mavlink_mission_item_int_t &ret_packet)
{
    if (packet.seq != 0 && // always allow HOME to be read
        packet.seq >= mission.num_commands()) {
        // try to educate the GCS on the actual size of the mission:
        mavlink_msg_mission_count_send(_link.get_chan(),
                                       msg.sysid,
                                       msg.compid,
                                       mission.num_commands(),
                                       MAV_MISSION_TYPE_MISSION);
        return MAV_MISSION_ERROR;
    }

    AP_Mission::Mission_Command cmd;

    // retrieve mission from eeprom
    if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
        return MAV_MISSION_ERROR;
    }

    if (!convert_Mission_Command_to_MISSION_ITEM_INT(cmd, ret_packet)) {
        return MAV_MISSION_ERROR;
    }

    // set packet's current field to 1 if this is the command being executed
    if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
        ret_packet.current = 1;
    } else {
        ret_packet.current = 0;
    }

    // set auto continue to 1
    ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

    ret_packet.command = cmd.id;

    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Waypoints::truncate(const mavlink_mission_count_t &packet)
{
    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);
}

bool MissionItemProtocol_Waypoints::clear_all_items()
{
    return mission.clear();
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::replace_item(const mavlink_mission_item_int_t &mission_item_int)
{
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = convert_MISSION_ITEM_INT_to_Mission_Command(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }
    if (!mission.replace_cmd(cmd.index, cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::append_item(const mavlink_mission_item_int_t &mission_item_int)
{
    // sanity check for DO_JUMP command
    AP_Mission::Mission_Command cmd;

    const MAV_MISSION_RESULT res = convert_MISSION_ITEM_INT_to_Mission_Command(mission_item_int, cmd);
    if (res != MAV_MISSION_ACCEPTED) {
        return res;
    }

    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= item_count() && cmd.content.jump.target > request_last) || cmd.content.jump.target == 0) {
            return MAV_MISSION_ERROR;
        }
    }

    if (!mission.add_cmd(cmd)) {
        return MAV_MISSION_ERROR;
    }
    return MAV_MISSION_ACCEPTED;
}

void MissionItemProtocol_Waypoints::complete(const GCS_MAVLINK &_link)
{
    _link.send_text(MAV_SEVERITY_INFO, "Flight plan received");
    AP::logger().Write_EntireMission();
}

void MissionItemProtocol_Waypoints::timeout()
{
    link->send_text(MAV_SEVERITY_WARNING, "Mission upload timeout");
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::sanity_check_params(
    const mavlink_mission_item_int_t& packet) {
    uint8_t nan_mask;
    switch (packet.command) {
        case MAV_CMD_NAV_WAYPOINT:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_LAND:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_TAKEOFF:
            nan_mask = ~(1 << 3); // param 4 can be nan
            break;
        case MAV_CMD_NAV_VTOL_LAND:
            nan_mask = ~((1 << 2) | (1 << 3)); // param 3 and 4 can be nan
            break;
        default:
            nan_mask = 0xff;
            break;
    }

    if (((nan_mask & (1 << 0)) && isnan(packet.param1)) ||
        isinf(packet.param1)) {
        return MAV_MISSION_INVALID_PARAM1;
    }
    if (((nan_mask & (1 << 1)) && isnan(packet.param2)) ||
        isinf(packet.param2)) {
        return MAV_MISSION_INVALID_PARAM2;
    }
    if (((nan_mask & (1 << 2)) && isnan(packet.param3)) ||
        isinf(packet.param3)) {
        return MAV_MISSION_INVALID_PARAM3;
    }
    if (((nan_mask & (1 << 3)) && isnan(packet.param4)) ||
        isinf(packet.param4)) {
        return MAV_MISSION_INVALID_PARAM4;
    }
    return MAV_MISSION_ACCEPTED;
}

//  return MAV_MISSION_ACCEPTED on success, MAV_MISSION_RESULT error on failure
MAV_MISSION_RESULT MissionItemProtocol_Waypoints::convert_MISSION_ITEM_INT_to_Mission_Command(
    const mavlink_mission_item_int_t& item_int,
    AP_Mission::Mission_Command& cmd)
{
    // command's position in mission list and mavlink id
    cmd.index = item_int.seq;
    cmd.id = item_int.command;
    cmd.content.location = {};

    MAV_MISSION_RESULT param_check = sanity_check_params(item_int);
    if (param_check != MAV_MISSION_ACCEPTED) {
        return param_check;
    }

    // command specific conversions from mavlink item_int to mission command
    switch (cmd.id) {

    case 0:
        // this is reserved for storing 16 bit command IDs
        return MAV_MISSION_INVALID;

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
    {
        /*
          the 15 byte limit means we can't fit both delay and radius
          in the cmd structure. When we expand the mission structure
          we can do this properly
         */
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters and pass by distance in meters
        uint16_t acp = item_int.param2;           // param 2 is acceptance radius in meters is held in low p1
        uint16_t passby = item_int.param3;        // param 3 is pass by distance in meters is held in high p1

        // limit to 255 so it does not wrap during the shift or mask operation
        passby = MIN(0xFF,passby);
        acp = MIN(0xFF,acp);

        cmd.p1 = (passby << 8) | (acp & 0x00FF);
#else
        // delay at waypoint in seconds (this is for copters???)
        cmd.p1 = item_int.param1;
#endif
    }
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        cmd.p1 = fabsf(item_int.param3);                  // store radius as 16bit since no other params are competing for space
        cmd.content.location.loiter_ccw = (item_int.param3 < 0);    // -1 = counter clockwise, +1 = clockwise
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
    {
        uint16_t num_turns = item_int.param1;              // param 1 is number of times to circle is held in low p1
        uint16_t radius_m = fabsf(item_int.param3);        // param 3 is radius in meters is held in high p1
        cmd.p1 = (radius_m<<8) | (num_turns & 0x00FF);   // store radius in high byte of p1, num turns in low byte of p1
        cmd.content.location.loiter_ccw = (item_int.param3 < 0);
        cmd.content.location.loiter_xtrack = (item_int.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    }
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        cmd.p1 = item_int.param1;                         // loiter time in seconds uses all 16 bits, 8bit seconds is too small. No room for radius.
        cmd.content.location.loiter_ccw = (item_int.param3 < 0);
        cmd.content.location.loiter_xtrack = (item_int.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        cmd.p1 = item_int.param1;                         // abort target altitude(m)  (plane only)
        cmd.content.location.loiter_ccw = is_negative(item_int.param4); // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        cmd.p1 = item_int.param1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        cmd.p1 = item_int.param1;                         // Climb/Descend
                        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
                        // 1 = Climb, cmd complete at or above indicated alt.
                        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        cmd.p1 = fabsf(item_int.param2);                  // param2 is radius in meters
        cmd.content.location.loiter_ccw = (item_int.param2 < 0);
        cmd.content.location.loiter_xtrack = (item_int.param4 > 0); // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        cmd.p1 = item_int.param1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        cmd.p1 = item_int.param1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 94
        cmd.content.nav_delay.seconds = item_int.param1; // delay in seconds
        cmd.content.nav_delay.hour_utc = item_int.param2;// absolute time's hour (utc)
        cmd.content.nav_delay.min_utc = item_int.param3;// absolute time's min (utc)
        cmd.content.nav_delay.sec_utc = item_int.param4; // absolute time's second (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        cmd.content.delay.seconds = item_int.param1;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        cmd.content.distance.meters = item_int.param1;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        cmd.content.yaw.angle_deg = item_int.param1;      // target angle in degrees
        cmd.content.yaw.turn_rate_dps = item_int.param2;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        cmd.content.yaw.direction = item_int.param3;      // -1 = ccw, +1 = cw
        cmd.content.yaw.relative_angle = item_int.param4; // lng=0: absolute angle provided, lng=1: relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        cmd.p1 = item_int.param1;                         // flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        cmd.content.jump.target = item_int.param1;        // jump-to command number
        cmd.content.jump.num_times = item_int.param2;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        cmd.content.speed.speed_type = item_int.param1;   // 0 = airspeed, 1 = ground speed
        cmd.content.speed.target_ms = item_int.param2;    // target speed in m/s
        cmd.content.speed.throttle_pct = item_int.param3; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:
        cmd.p1 = item_int.param1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        cmd.content.relay.num = item_int.param1;          // relay number
        cmd.content.relay.state = item_int.param2;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        cmd.content.repeat_relay.num = item_int.param1;           // relay number
        cmd.content.repeat_relay.repeat_count = item_int.param2;  // count
        cmd.content.repeat_relay.cycle_time = item_int.param3;    // time converted from seconds to milliseconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        cmd.content.servo.channel = item_int.param1;      // channel
        cmd.content.servo.pwm = item_int.param2;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        cmd.content.repeat_servo.channel = item_int.param1;      // channel
        cmd.content.repeat_servo.pwm = item_int.param2;          // PWM
        cmd.content.repeat_servo.repeat_count = item_int.param3; // count
        cmd.content.repeat_servo.cycle_time = item_int.param4;   // time in seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        cmd.p1 = item_int.param1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        cmd.content.digicam_configure.shooting_mode = item_int.param1;
        cmd.content.digicam_configure.shutter_speed = item_int.param2;
        cmd.content.digicam_configure.aperture = item_int.param3;
        cmd.content.digicam_configure.ISO = item_int.param4;
        cmd.content.digicam_configure.exposure_type = item_int.x;
        cmd.content.digicam_configure.cmd_id = item_int.y;
        cmd.content.digicam_configure.engine_cutoff_time = item_int.z;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        cmd.content.digicam_control.session = item_int.param1;
        cmd.content.digicam_control.zoom_pos = item_int.param2;
        cmd.content.digicam_control.zoom_step = item_int.param3;
        cmd.content.digicam_control.focus_lock = item_int.param4;
        cmd.content.digicam_control.shooting_cmd = item_int.x;
        cmd.content.digicam_control.cmd_id = item_int.y;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        cmd.content.mount_control.pitch = item_int.param1;
        cmd.content.mount_control.roll = item_int.param2;
        cmd.content.mount_control.yaw = item_int.param3;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        cmd.content.cam_trigg_dist.meters = item_int.param1;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        cmd.p1 = item_int.param1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                         // MAV ID: 208
        cmd.p1 = item_int.param1;                        // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        cmd.p1 = item_int.param1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        cmd.content.gripper.num = item_int.param1;        // gripper number
        cmd.content.gripper.action = item_int.param2;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        cmd.p1 = item_int.param1;                         // max time in seconds the external controller will be allowed to control the vehicle
        cmd.content.guided_limits.alt_min = item_int.param2;  // min alt below which the command will be aborted.  0 for no lower alt limit
        cmd.content.guided_limits.alt_max = item_int.param3;  // max alt above which the command will be aborted.  0 for no upper alt limit
        cmd.content.guided_limits.horiz_max = item_int.param4;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:                    // MAV ID: 211
        cmd.p1 = item_int.param1;                         // disable=0 enable=1
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        cmd.content.altitude_wait.altitude = item_int.param1;
        cmd.content.altitude_wait.descent_rate = item_int.param2;
        cmd.content.altitude_wait.wiggle_time = item_int.param3;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        cmd.content.do_vtol_transition.target_state = item_int.param1;
        break;

    case MAV_CMD_DO_SET_REVERSE:
        cmd.p1 = item_int.param1; // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        cmd.content.do_engine_control.start_control = (item_int.param1>0);
        cmd.content.do_engine_control.cold_start = (item_int.param2>0);
        cmd.content.do_engine_control.height_delay_cm = item_int.param3*100;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        cmd.p1 = item_int.param1*100; // copy max-descend parameter (m->cm)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        cmd.content.set_yaw_speed.angle_deg = item_int.param1;        // target angle in degrees
        cmd.content.set_yaw_speed.speed = item_int.param2;            // speed in meters/second
        cmd.content.set_yaw_speed.relative_angle = item_int.param3;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:                              // MAV ID: 42600
        cmd.content.winch.num = item_int.param1;          // winch number
        cmd.content.winch.action = item_int.param2;       // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        cmd.content.winch.release_length = item_int.param3;   // cable distance to unwind in meters, negative numbers to wind in cable
        cmd.content.winch.release_rate = item_int.param4; // release rate in meters/second
        break;

    default:
        // unrecognised command
        return MAV_MISSION_UNSUPPORTED;
    }

    // copy location from mavlink to command
    if (AP_Mission::stored_in_location(cmd.id)) {

        // sanity check location
        if (!check_lat(item_int.x)) {
            return MAV_MISSION_INVALID_PARAM5_X;
        }
        if (!check_lng(item_int.y)) {
            return MAV_MISSION_INVALID_PARAM6_Y;
        }
        if (isnan(item_int.z) || fabsf(item_int.z) >= LOCATION_ALT_MAX_M) {
            return MAV_MISSION_INVALID_PARAM7;
        }

        cmd.content.location.lat = item_int.x;
        cmd.content.location.lng = item_int.y;

        cmd.content.location.alt = item_int.z * 100.0f;       // convert item_int's alt (m) to cmd alt (cm)

        switch (item_int.frame) {

        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
            cmd.content.location.relative_alt = 0;
            break;

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            cmd.content.location.relative_alt = 1;
            break;

#if AP_TERRAIN_AVAILABLE
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            // we mark it as a relative altitude, as it doesn't have
            // home alt added
            cmd.content.location.relative_alt = 1;
            // mark altitude as above terrain, not above home
            cmd.content.location.terrain_alt = 1;
            break;
#endif

        default:
            return MAV_MISSION_UNSUPPORTED_FRAME;
        }
    }

    // if we got this far then it must have been successful
    return MAV_MISSION_ACCEPTED;
}

MAV_MISSION_RESULT MissionItemProtocol_Waypoints::convert_COMMAND_LONG_to_Mission_Command(
    const mavlink_command_long_t& packet,
    AP_Mission::Mission_Command& cmd)
{
    mavlink_mission_item_int_t miss_item{};

    miss_item.param1 = packet.param1;
    miss_item.param2 = packet.param2;
    miss_item.param3 = packet.param3;
    miss_item.param4 = packet.param4;

    miss_item.command = packet.command;
    miss_item.target_system = packet.target_system;
    miss_item.target_component = packet.target_component;

    return convert_MISSION_ITEM_INT_to_Mission_Command(miss_item, cmd);
}

bool MissionItemProtocol_Waypoints::convert_Mission_Command_to_MISSION_ITEM_INT(
    const AP_Mission::Mission_Command& cmd,
    mavlink_mission_item_int_t& item_int)
{
    // command's position in mission list and mavlink id
    item_int.seq = cmd.index;
    item_int.command = cmd.id;

    // set defaults
    item_int.current = 0;     // 1 if we are passing back the mission command that is currently being executed
    item_int.param1 = 0;
    item_int.param2 = 0;
    item_int.param3 = 0;
    item_int.param4 = 0;
    item_int.autocontinue = 1;

    // command specific conversions from mission command to mavlink item_int
    switch (cmd.id) {
    case 0:
        // this is reserved for 16 bit command IDs
        return false;

    case MAV_CMD_NAV_WAYPOINT:                          // MAV ID: 16
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
        // acceptance radius in meters

        item_int.param2 = LOWBYTE(cmd.p1);        // param 2 is acceptance radius in meters is held in low p1
        item_int.param3 = HIGHBYTE(cmd.p1);       // param 3 is pass by distance in meters is held in high p1
#else
        // delay at waypoint in seconds
        item_int.param1 = cmd.p1;
#endif
        break;

    case MAV_CMD_NAV_LOITER_UNLIM:                      // MAV ID: 17
        item_int.param3 = (float)cmd.p1;
        if (cmd.content.location.loiter_ccw) {
            item_int.param3 *= -1;
        }
        break;

    case MAV_CMD_NAV_LOITER_TURNS:                      // MAV ID: 18
        item_int.param1 = LOWBYTE(cmd.p1);                // number of times to circle is held in low byte of p1
        item_int.param3 = HIGHBYTE(cmd.p1);               // radius is held in high byte of p1
        if (cmd.content.location.loiter_ccw) {
            item_int.param3 = -item_int.param3;
        }
        item_int.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_LOITER_TIME:                       // MAV ID: 19
        item_int.param1 = cmd.p1;                         // loiter time in seconds
        if (cmd.content.location.loiter_ccw) {
            item_int.param3 = -1;
        } else {
            item_int.param3 = 1;
        }
        item_int.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:                  // MAV ID: 20
        break;

    case MAV_CMD_NAV_LAND:                              // MAV ID: 21
        item_int.param1 = cmd.p1;                        // abort target altitude(m)  (plane only)
        item_int.param4 = cmd.content.location.loiter_ccw ? -1 : 1; // yaw direction, (plane deepstall only)
        break;

    case MAV_CMD_NAV_TAKEOFF:                           // MAV ID: 22
        item_int.param1 = cmd.p1;                         // minimum pitch (plane only)
        break;

    case MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:           // MAV ID: 30
        item_int.param1 = cmd.p1;                         // Climb/Descend
                        // 0 = Neutral, cmd complete at +/- 5 of indicated alt.
                        // 1 = Climb, cmd complete at or above indicated alt.
                        // 2 = Descend, cmd complete at or below indicated alt.
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:                     // MAV ID: 31
        item_int.param2 = cmd.p1;                        // loiter radius(m)
        if (cmd.content.location.loiter_ccw) {
            item_int.param2 = -item_int.param2;
        }
        item_int.param4 = cmd.content.location.loiter_xtrack; // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        break;

    case MAV_CMD_NAV_SPLINE_WAYPOINT:                   // MAV ID: 82
        item_int.param1 = cmd.p1;                         // delay at waypoint in seconds
        break;

    case MAV_CMD_NAV_GUIDED_ENABLE:                     // MAV ID: 92
        item_int.param1 = cmd.p1;                         // on/off. >0.5 means "on", hand-over control to external controller
        break;

    case MAV_CMD_NAV_DELAY:                            // MAV ID: 94
        item_int.param1 = cmd.content.nav_delay.seconds; // delay in seconds
        item_int.param2 = cmd.content.nav_delay.hour_utc; // absolute time's day of week (utc)
        item_int.param3 = cmd.content.nav_delay.min_utc; // absolute time's hour (utc)
        item_int.param4 = cmd.content.nav_delay.sec_utc; // absolute time's min (utc)
        break;

    case MAV_CMD_CONDITION_DELAY:                       // MAV ID: 112
        item_int.param1 = cmd.content.delay.seconds;      // delay in seconds
        break;

    case MAV_CMD_CONDITION_DISTANCE:                    // MAV ID: 114
        item_int.param1 = cmd.content.distance.meters;    // distance in meters from next waypoint
        break;

    case MAV_CMD_CONDITION_YAW:                         // MAV ID: 115
        item_int.param1 = cmd.content.yaw.angle_deg;      // target angle in degrees
        item_int.param2 = cmd.content.yaw.turn_rate_dps;  // 0 = use default turn rate otherwise specific turn rate in deg/sec
        item_int.param3 = cmd.content.yaw.direction;      // -1 = ccw, +1 = cw
        item_int.param4 = cmd.content.yaw.relative_angle; // 0 = absolute angle provided, 1 = relative angle provided
        break;

    case MAV_CMD_DO_SET_MODE:                           // MAV ID: 176
        item_int.param1 = cmd.p1;                         // set flight mode identifier
        break;

    case MAV_CMD_DO_JUMP:                               // MAV ID: 177
        item_int.param1 = cmd.content.jump.target;        // jump-to command number
        item_int.param2 = cmd.content.jump.num_times;     // repeat count
        break;

    case MAV_CMD_DO_CHANGE_SPEED:                       // MAV ID: 178
        item_int.param1 = cmd.content.speed.speed_type;   // 0 = airspeed, 1 = ground speed
        item_int.param2 = cmd.content.speed.target_ms;    // speed in m/s
        item_int.param3 = cmd.content.speed.throttle_pct; // throttle as a percentage from 0 ~ 100%
        break;

    case MAV_CMD_DO_SET_HOME:                           // MAV ID: 179
        item_int.param1 = cmd.p1;                         // p1=0 means use current location, p=1 means use provided location
        break;

    case MAV_CMD_DO_SET_RELAY:                          // MAV ID: 181
        item_int.param1 = cmd.content.relay.num;          // relay number
        item_int.param2 = cmd.content.relay.state;        // 0:off, 1:on
        break;

    case MAV_CMD_DO_REPEAT_RELAY:                       // MAV ID: 182
        item_int.param1 = cmd.content.repeat_relay.num;           // relay number
        item_int.param2 = cmd.content.repeat_relay.repeat_count;  // count
        item_int.param3 = cmd.content.repeat_relay.cycle_time;    // time in seconds
        break;

    case MAV_CMD_DO_SET_SERVO:                          // MAV ID: 183
        item_int.param1 = cmd.content.servo.channel;      // channel
        item_int.param2 = cmd.content.servo.pwm;          // PWM
        break;

    case MAV_CMD_DO_REPEAT_SERVO:                       // MAV ID: 184
        item_int.param1 = cmd.content.repeat_servo.channel;       // channel
        item_int.param2 = cmd.content.repeat_servo.pwm;           // PWM
        item_int.param3 = cmd.content.repeat_servo.repeat_count;  // count
        item_int.param4 = cmd.content.repeat_servo.cycle_time;    // time in milliseconds converted to seconds
        break;

    case MAV_CMD_DO_LAND_START:                         // MAV ID: 189
        break;

    case MAV_CMD_DO_GO_AROUND:                          // MAV ID: 191
        break;

    case MAV_CMD_DO_SET_ROI:                            // MAV ID: 201
        item_int.param1 = cmd.p1;                         // 0 = no roi, 1 = next waypoint, 2 = waypoint number, 3 = fixed location, 4 = given target (not supported)
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:                  // MAV ID: 202
        item_int.param1 = cmd.content.digicam_configure.shooting_mode;
        item_int.param2 = cmd.content.digicam_configure.shutter_speed;
        item_int.param3 = cmd.content.digicam_configure.aperture;
        item_int.param4 = cmd.content.digicam_configure.ISO;
        item_int.x = cmd.content.digicam_configure.exposure_type;
        item_int.y = cmd.content.digicam_configure.cmd_id;
        item_int.z = cmd.content.digicam_configure.engine_cutoff_time;
        break;

    case MAV_CMD_DO_DIGICAM_CONTROL:                    // MAV ID: 203
        item_int.param1 = cmd.content.digicam_control.session;
        item_int.param2 = cmd.content.digicam_control.zoom_pos;
        item_int.param3 = cmd.content.digicam_control.zoom_step;
        item_int.param4 = cmd.content.digicam_control.focus_lock;
        item_int.x = cmd.content.digicam_control.shooting_cmd;
        item_int.y = cmd.content.digicam_control.cmd_id;
        break;

    case MAV_CMD_DO_MOUNT_CONTROL:                      // MAV ID: 205
        item_int.param1 = cmd.content.mount_control.pitch;
        item_int.param2 = cmd.content.mount_control.roll;
        item_int.param3 = cmd.content.mount_control.yaw;
        break;

    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:                 // MAV ID: 206
        item_int.param1 = cmd.content.cam_trigg_dist.meters;  // distance between camera shots in meters
        break;

    case MAV_CMD_DO_FENCE_ENABLE:                       // MAV ID: 207
        item_int.param1 = cmd.p1;                         // action 0=disable, 1=enable
        break;

    case MAV_CMD_DO_PARACHUTE:                          // MAV ID: 208
        item_int.param1 = cmd.p1;                         // action 0=disable, 1=enable, 2=release.  See PARACHUTE_ACTION enum
        break;

    case MAV_CMD_DO_INVERTED_FLIGHT:                    // MAV ID: 210
        item_int.param1 = cmd.p1;                         // normal=0 inverted=1
        break;

    case MAV_CMD_DO_GRIPPER:                            // MAV ID: 211
        item_int.param1 = cmd.content.gripper.num;        // gripper number
        item_int.param2 = cmd.content.gripper.action;     // action 0=release, 1=grab.  See GRIPPER_ACTION enum
        break;

    case MAV_CMD_DO_GUIDED_LIMITS:                      // MAV ID: 222
        item_int.param1 = cmd.p1;                         // max time in seconds the external controller will be allowed to control the vehicle
        item_int.param2 = cmd.content.guided_limits.alt_min;  // min alt below which the command will be aborted.  0 for no lower alt limit
        item_int.param3 = cmd.content.guided_limits.alt_max;  // max alt above which the command will be aborted.  0 for no upper alt limit
        item_int.param4 = cmd.content.guided_limits.horiz_max;// max horizontal distance the vehicle can move before the command will be aborted.  0 for no horizontal limit
        break;

    case MAV_CMD_DO_AUTOTUNE_ENABLE:
        item_int.param1 = cmd.p1;                         // disable=0 enable=1
        break;

    case MAV_CMD_DO_SET_REVERSE:
        item_int.param1 = cmd.p1;   // 0 = forward, 1 = reverse
        break;

    case MAV_CMD_NAV_ALTITUDE_WAIT:                     // MAV ID: 83
        item_int.param1 = cmd.content.altitude_wait.altitude;
        item_int.param2 = cmd.content.altitude_wait.descent_rate;
        item_int.param3 = cmd.content.altitude_wait.wiggle_time;
        break;

    case MAV_CMD_NAV_VTOL_TAKEOFF:
        break;

    case MAV_CMD_NAV_VTOL_LAND:
        break;

    case MAV_CMD_DO_VTOL_TRANSITION:
        item_int.param1 = cmd.content.do_vtol_transition.target_state;
        break;

    case MAV_CMD_DO_ENGINE_CONTROL:
        item_int.param1 = cmd.content.do_engine_control.start_control?1:0;
        item_int.param2 = cmd.content.do_engine_control.cold_start?1:0;
        item_int.param3 = cmd.content.do_engine_control.height_delay_cm*0.01f;
        break;

    case MAV_CMD_NAV_PAYLOAD_PLACE:
        item_int.param1 = cmd.p1/100.0f; // copy max-descend parameter (m->cm)
        break;

    case MAV_CMD_NAV_SET_YAW_SPEED:
        item_int.param1 = cmd.content.set_yaw_speed.angle_deg;        // target angle in degrees
        item_int.param2 = cmd.content.set_yaw_speed.speed;            // speed in meters/second
        item_int.param3 = cmd.content.set_yaw_speed.relative_angle;   // 0 = absolute angle, 1 = relative angle
        break;

    case MAV_CMD_DO_WINCH:
        item_int.param1 = cmd.content.winch.num;              // winch number
        item_int.param2 = cmd.content.winch.action;           // action (0 = relax, 1 = length control, 2 = rate control).  See WINCH_ACTION enum
        item_int.param3 = cmd.content.winch.release_length;   // cable distance to unwind in meters, negative numbers to wind in cable
        item_int.param4 = cmd.content.winch.release_rate;     // release rate in meters/second
        break;

    default:
        // unrecognised command
        return false;
    }

    // copy location from mavlink to command
    if (AP_Mission::stored_in_location(cmd.id)) {
        item_int.x = cmd.content.location.lat;
        item_int.y = cmd.content.location.lng;

        item_int.z = cmd.content.location.alt / 100.0f;   // cmd alt in cm to m
        if (cmd.content.location.relative_alt) {
            item_int.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
        }else{
            item_int.frame = MAV_FRAME_GLOBAL;
        }
#if AP_TERRAIN_AVAILABLE
        if (cmd.content.location.terrain_alt) {
            // this is a above-terrain altitude
            if (!cmd.content.location.relative_alt) {
                // refuse to return non-relative terrain mission
                // items. Internally we do have these, and they
                // have home.alt added, but we should never be
                // returning them to the GCS, as the GCS doesn't know
                // our home.alt, so it would have no way to properly
                // interpret it
                return false;
            }
            item_int.z = cmd.content.location.alt * 0.01f;
            item_int.frame = MAV_FRAME_GLOBAL_TERRAIN_ALT;
        }
#else
        // don't ever return terrain mission items if no terrain support
        if (cmd.content.location.terrain_alt) {
            return false;
        }
#endif
    }

    // if we got this far then it must have been successful
    return true;
}
