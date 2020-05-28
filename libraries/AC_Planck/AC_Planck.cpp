#include <AC_Planck/AC_Planck.h>
#include <AP_HAL/AP_HAL.h>
#include "../ArduCopter/defines.h"

void AC_Planck::handle_planck_mavlink_msg(const mavlink_channel_t &chan, const mavlink_message_t *mav_msg,
    AP_AHRS &ahrs)
{
  switch(mav_msg->msgid)
  {
     case MAVLINK_MSG_ID_PLANCK_STATUS:
     {
        _chan = chan; //Set the channel based on the incoming status message
        mavlink_planck_status_t ps;
        mavlink_msg_planck_status_decode(mav_msg, &ps);
        _status.timestamp_ms = AP_HAL::millis();
        _status.takeoff_ready = (bool)ps.takeoff_ready;
        _status.land_ready = (bool)ps.land_ready;
        _status.commbox_ok = (bool)(ps.failsafe & 0x01);
        _status.commbox_gps_ok = (bool)(ps.failsafe & 0x02);
        _status.tracking_tag = (bool)(ps.status & 0x01);
        _status.tracking_commbox_gps = (bool)(ps.status & 0x02);
        _status.takeoff_complete = (bool)ps.takeoff_complete;
        _status.at_location = (bool)ps.at_location;

        //_was_at_location is special, as it is only triggered once per event
        //on the planck side. set the flag but also the oneshot value
        if(!_was_at_location && _status.at_location) {
          _was_at_location = true;
        }
        break;
    }

     case MAVLINK_MSG_ID_PLANCK_CMD_MSG:
     {
        mavlink_planck_cmd_msg_t pc;
        mavlink_msg_planck_cmd_msg_decode(mav_msg, &pc);

        //position data
        _cmd.pos.lat = pc.lat;
        _cmd.pos.lng = pc.lon;
        _cmd.pos.alt = pc.alt * 100; //m->cm

         switch(pc.frame) {
            case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
              _cmd.pos.relative_alt = true;
              _cmd.pos.terrain_alt = false;
              break;
            case MAV_FRAME_GLOBAL_TERRAIN_ALT:
            case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
              _cmd.pos.relative_alt = true;
              _cmd.pos.terrain_alt = true;
              break;
            case MAV_FRAME_GLOBAL:
            case MAV_FRAME_GLOBAL_INT:
            default:
              // Copter does not support navigation to absolute altitudes. This convert the WGS84 altitude
              // to a home-relative altitude before passing it to the navigation controller
              _cmd.pos.alt -= ahrs.get_home().alt;
              _cmd.pos.relative_alt = true;
              _cmd.pos.terrain_alt = false;
              break;
        }

        //velocity
        _cmd.vel_cms.x = pc.vel[0] * 100.;
        _cmd.vel_cms.y = pc.vel[1] * 100.;
        _cmd.vel_cms.z = pc.vel[2] * 100.;

        //acceleration
        _cmd.accel_cmss.x = pc.acc[0] * 100.;
        _cmd.accel_cmss.y = pc.acc[1] * 100.;
        _cmd.accel_cmss.z = pc.acc[2] * 100.;

        //Attitude
        _cmd.att_cd.x = ToDeg(pc.att[0]) * 100.;
        _cmd.att_cd.y = ToDeg(pc.att[1]) * 100.;
        _cmd.att_cd.z = ToDeg(pc.att[2]) * 100.;

        //Determine which values are good
        bool use_pos = (pc.type_mask & 0x0007) == 0x0007;
        bool use_vel = (pc.type_mask & 0x0038) == 0x0038;
        bool use_vz  = (pc.type_mask & 0x0020) == 0x0020;
        bool use_acc = (pc.type_mask & 0x01C0) == 0x01C0;
        bool use_att = (pc.type_mask & 0x0E00) == 0x0E00;
        bool use_y   = (pc.type_mask & 0x0800) == 0x0800;
        bool use_yr  = (pc.type_mask & 0x1000) == 0x1000;

        _cmd.is_yaw_rate = use_yr;

        //Determine the command type based on the typemask
        //If position bits are set, this is a position command
        if(use_pos && !use_vel)
          _cmd.type = POSITION;

        //If position and velocity are set, this is a posvel
        else if(use_pos && use_vel)
          _cmd.type = POSVEL;

        //If velocity is set, this is a velocity command
        else if(use_vel)
          _cmd.type = VELOCITY;

        //If attitude and vz and yaw/yawrate are set, this is an attitude command
        else if(use_vz && !use_acc && use_att && (use_y || use_yr))
          _cmd.type = ATTITUDE;

        //If accel and vz and yaw/yawrate are set, this is an accel command
        else if(use_vz && use_acc && !use_att && (use_y || use_yr))
          _cmd.type = ACCEL;

        //Otherwise we don't know what this is
        else
          _cmd.type = NONE;

        //This is a new command
        _cmd.is_new = true;
        break;
    }

    default:
      break;
  }
}

void AC_Planck::request_takeoff(const float alt)
{
  //Send a takeoff command message to planck
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,         //uint8_t target_system
    PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
    PLANCK_CMD_REQ_TAKEOFF,//uint8_t type
    alt,                   //float param1
    0,0,0,0,0);
}

void AC_Planck::request_alt_change(const float alt)
{
  //Only altitude is valid
  uint8_t valid = 0b00000100;
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,
    PLANCK_CTRL_COMP_ID,
    PLANCK_CMD_REQ_MOVE_TARGET,
    (float)valid,     //param1
    0,                //param2
    0,                //param3
    alt,              //param4
    0,                //param5
    false);           //param6
}

void AC_Planck::request_rtb(const float alt, const float rate_up, const float rate_down, const float rate_xy)
{
  //Send an RTL command message to planck
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,         //uint8_t target_system
    PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
    PLANCK_CMD_REQ_RTB,    //uint8_t type
    alt,                   //float param1
    rate_up,               //float param2
    rate_down,             //float param3
    rate_xy,               //float param4
    0,0);
}

void AC_Planck::request_land(const float descent_rate)
{
  //Send a land command message to planck
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,         //uint8_t target_system
    PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
    PLANCK_CMD_REQ_LAND,   //uint8_t type
    descent_rate,          //float param1
    0,0,0,0,0);
}

//Move the current tracking target, either to an absolute offset or by a rate
void AC_Planck::request_move_target(const Vector3f offset_cmd_NED, const bool is_rate)
{
  //all directions and are valid
  uint8_t valid = 0b00000111;
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,
    PLANCK_CTRL_COMP_ID,
    PLANCK_CMD_REQ_MOVE_TARGET,
    (float)valid,     //param1
    offset_cmd_NED.x, //param2
    offset_cmd_NED.y, //param3
    offset_cmd_NED.z, //param4
    is_rate,          //param5
    0);               //param6

  //If the target has moved, the _was_at_location flag must go false until we
  //hear otherwise from planck
  _was_at_location = false;
}

void AC_Planck::stop_commanding(void)
{
  mavlink_msg_planck_cmd_request_send(
    _chan,
    mavlink_system.sysid,         //uint8_t target_system
    PLANCK_CTRL_COMP_ID,   //uint8_t target_component,
    PLANCK_CMD_REQ_STOP,   //uint8_t type
    0,0,0,0,0,0);
}

//Get an accel, yaw, z_rate command
bool AC_Planck::get_accel_yaw_zrate_cmd(Vector3f &accel_cmss, float &yaw_cd, float &vz_cms, bool &is_yaw_rate)
{
  if(!_cmd.is_new) return false;
  accel_cmss = _cmd.accel_cmss;
  yaw_cd = _cmd.att_cd.z;
  vz_cms = _cmd.vel_cms.z;
  is_yaw_rate = _cmd.is_yaw_rate;
  _cmd.is_new = false;
  return true;
}

//Get an attitude command
bool AC_Planck::get_attitude_zrate_cmd(Vector3f &att_cd, float &vz_cms, bool &is_yaw_rate)
{
  if(!_cmd.is_new) return false;
  att_cd = _cmd.att_cd;
  vz_cms = _cmd.vel_cms.z;
  is_yaw_rate = _cmd.is_yaw_rate;
  _cmd.is_new = false;
  return true;
}

//Get a velocity, yaw command
bool AC_Planck::get_velocity_cmd(Vector3f &vel_cms)
{
  if(!_cmd.is_new) return false;
  vel_cms = _cmd.vel_cms;
  _cmd.is_new = false;
  return true;
}

//Get a position command
bool AC_Planck::get_position_cmd(Location &loc)
{
  if(!_cmd.is_new) return false;
  loc = _cmd.pos;
  _cmd.is_new = false;
  return true;
}

//Get a position, velocity cmd
bool AC_Planck::get_posvel_cmd(Location &loc, Vector3f &vel_cms)
{
  if(!_cmd.is_new) return false;
  loc = _cmd.pos;
  vel_cms = _cmd.vel_cms;
  _cmd.is_new = false;
  return true;
}
