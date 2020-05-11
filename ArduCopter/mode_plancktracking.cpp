#include "Copter.h"

bool ModePlanckTracking::init_without_RTB_request(bool ignore_checks) {
    //For initialization, if the GPS is bad but we have a tag detection,
    //just use GUIDED_NOGPS, as we don't want to require the use of GPS for
    //GPS-denied operation.  Subsequent commands will be accel/attitude based.
    if(!copter.position_ok() && copter.planck_interface.get_tag_tracking_state()) {
      //Set the angle to zero and a zero z rate; this prevents an initial drop
      ModeGuided::set_angle(Quaternion(),0,true,0);
      return copter.mode_guided_nogps.init(ignore_checks);
    }

    //Otherwise, use GUIDED
    return ModeGuided::init(ignore_checks);
}

bool ModePlanckTracking::init(bool ignore_checks){
    //If the copter is currently in-flight, entry into this mode indicates
    //that the user would like to return to the tag tracking, so run RTB
    if(!copter.ap.land_complete)
    {
        //Return home rate is either WPNAV_SPEED or RTL_SPEED, if specified
        float rate_xy_cms = g.rtl_speed_cms != 0 ? g.rtl_speed_cms : copter.wp_nav->get_default_speed_xy();

        copter.planck_interface.request_rtb(
          (float)copter.g.rtl_altitude/100.,
          copter.pos_control->get_max_speed_up()/100.,
          copter.pos_control->get_max_speed_down()/100.,
          rate_xy_cms/100.);
    }

    //Initialize the GUIDED methods
    return init_without_RTB_request(ignore_checks);
}

void ModePlanckTracking::run() {

    //If there is new command data, send it to Guided
    if(copter.planck_interface.new_command_available()) {
        switch(copter.planck_interface.get_cmd_type()) {
          //Set guided mode attitude/velocity commands
          case copter.planck_interface.ACCEL:
          {
              Vector3f accel_cmss;
              float yaw_cd;
              float vz_cms;
              bool is_yaw_rate;

              bool good_cmd = copter.planck_interface.get_accel_yaw_zrate_cmd(
                  accel_cmss, yaw_cd, vz_cms, is_yaw_rate
              );

              if(!good_cmd) {
                  accel_cmss.x = accel_cmss.y = yaw_cd = vz_cms = 0;
                  is_yaw_rate = true;
              }

              //Turn accel into lean angles
              float roll_cd, pitch_cd;
              copter.pos_control->accel_to_lean_angles(
                accel_cmss.x,
                accel_cmss.y,
                roll_cd,
                pitch_cd);

              //If we are in WINGMAN mode, the user controls yaw, even though
              //we might be getting attitude/acceleration commands
              if(copter.flightmode == &copter.mode_planckwingman && !copter.failsafe.radio)
              {
                  // get pilot's desired yaw rate
                  float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
                  yaw_cd = pilot_yaw_rate_cds;
                  is_yaw_rate = true;
              }

              //Convert this to quaternions, yaw rates
              Quaternion q;
              q.from_euler(ToRad(roll_cd/100.), ToRad(pitch_cd/100.), ToRad(yaw_cd/100.));
              float yaw_rate_rads = ToRad(yaw_cd / 100.);

              //Update the GUIDED mode controller
              ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
              break;
          }

          case copter.planck_interface.ATTITUDE:
          {
              Vector3f att_cd;
              float vz_cms;
              bool is_yaw_rate;

              bool good_cmd = copter.planck_interface.get_attitude_zrate_cmd(
                  att_cd, vz_cms, is_yaw_rate
              );

              if(!good_cmd) {
                  att_cd.x = att_cd.y = att_cd.z = vz_cms = 0;
                  is_yaw_rate = true;
              }

              //If we are in WINGMAN mode, the user controls yaw, even though
              //we might be getting attitude/acceleration commands
              if(copter.flightmode == &copter.mode_planckwingman && !copter.failsafe.radio)
              {
                  // get pilot's desired yaw rate
                  float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
                  att_cd.z = pilot_yaw_rate_cds;
                  is_yaw_rate = true;
              }

              //Convert this to quaternions, yaw rates
              Quaternion q;
              q.from_euler(ToRad(att_cd.x/100.), ToRad(att_cd.y/100.), ToRad(att_cd.z/100.));
              float yaw_rate_rads = ToRad(att_cd.z / 100.);

              //Update the GUIDED mode controller
              ModeGuided::set_angle(q,vz_cms,is_yaw_rate,yaw_rate_rads);
              break;
          }

          case copter.planck_interface.VELOCITY:
          {
              Vector3f vel_cmd;

              bool good_cmd = copter.planck_interface.get_velocity_cmd(vel_cmd);

              if(!good_cmd) {
                  vel_cmd.x = vel_cmd.y = vel_cmd.z = 0;
              }

              ModeGuided::set_velocity(vel_cmd);
              break;
          }

          case copter.planck_interface.POSITION:
          {
              Location loc_cmd;
              copter.planck_interface.get_position_cmd(loc_cmd);
              ModeGuided::set_destination(loc_cmd);
              break;
          }

          case copter.planck_interface.POSVEL:
          {
              Location loc_cmd;
              Vector3f vel_cmd;
              float yaw_cmd;

              bool good_cmd = copter.planck_interface.get_posvel_cmd(
                loc_cmd,
                vel_cmd,
                yaw_cmd);

              //Set a zero velocity if this is a bad command
              if(!good_cmd)
              {
                  vel_cmd.x = vel_cmd.y = vel_cmd.z = 0;
                  ModeGuided::set_velocity(vel_cmd);
              }
              else
              {
                  //GUIDED posvel doesn't account for terrain altitude; get the
                  //terrain-alt shifted home-relative altitude if this is a
                  //terrain-relative command
                  Vector3f pos_cmd;
                  if(!loc_cmd.get_vector_from_origin_NEU(pos_cmd)){
                      //Should never happen
                      break;
                  }
                  if(loc_cmd.terrain_alt) {
                      Location loc(loc_cmd);
                      int32_t new_alt_cm = pos_cmd.z;
                      if(!loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, new_alt_cm)) {
                          AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
                      } else {
                          pos_cmd.z = new_alt_cm;
                      }
                  }

                  ModeGuided::set_destination_posvel(pos_cmd,vel_cmd,true,yaw_cmd);
              }
              break;
          }

          default:
            break;
      }
    }

    //Run the guided mode controller
    ModeGuided::run(true); //use high-jerk
}

bool ModePlanckTracking::do_user_takeoff_start(float final_alt_above_home)
{
    // Check if planck is ready
    if(!copter.planck_interface.ready_for_takeoff())
      return false;

    // Tell planck to start commanding
    copter.planck_interface.request_takeoff(final_alt_above_home/100.);

    // initialise yaw
    auto_yaw.set_mode(AUTO_YAW_HOLD);

    // clear i term when we're taking off
    set_throttle_takeoff();

    return true;
}

//Allow arming if planck is ready for takeooff and this is a GCS command
bool ModePlanckTracking::allows_arming(bool from_gcs) const
{
    if(!from_gcs) return false;
    if(!copter.planck_interface.ready_for_takeoff())
    {
        if(!copter.planck_interface.get_tag_tracking_state())
        {
            copter.gcs().send_text(MAV_SEVERITY_CRITICAL,
              "Arm: Planck not tracking tag");
        }
        else
        {
            copter.gcs().send_text(MAV_SEVERITY_CRITICAL,
              "Arm: Planck not ready for takeoff");
        }

        return false;
    }

    //Planck commbox GPS checks are checked in AP_Arming

    return true;
}

