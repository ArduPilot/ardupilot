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

    // check if gimbal steering needs to controlling the vehicle yaw
    AP_Mount *mount = AP::mount();
    bool is_payload_yaw_rate = false;
    bool use_payload_yaw_rate = false;
    float payload_yaw_rate_cds = 0;

    //Check for tether high tension
    bool high_tension = copter.planck_interface.is_tether_high_tension() || copter.planck_interface.is_tether_timed_out();
    bool use_positive_throttle = high_tension;

    // Don't use/set heading command from gimbal if tether is in high tension.
    if(!high_tension)
    {
      // If the gimbal is in vehicle_yaw_follows_gimbal mode, it sends a heading offset command,
      // so we add that offset to the current vehicle heading to obtain the desired absoute heading.
      // Note that the getter method "get_follow_yaw_rate()" is poorly named; it is not a rate,
      // it's the differene between vehicle and gimbal heading.
      if(mount != nullptr) {
        if(mount->mount_yaw_follow_mode == AP_Mount::vehicle_yaw_follows_gimbal) {

          //only set new yaw command if payload data is recent (1/2 sec)
          if (AP_HAL::micros64() - mount->get_last_payload_update_us() < 500000)
          {
            float angle_deg = mount->get_follow_yaw_rate() + degrees(copter.ahrs.get_yaw());
            int8_t direction = 1;
            bool relative_angle = false;

            // update auto_yaw private variable _fixed_yaw
            copter.flightmode->auto_yaw.set_fixed_yaw(
                  angle_deg,
                  0, // use default angle change rate
                  direction,
                  relative_angle);
          }
          is_payload_yaw_rate = false;
        }
        else if(!mount->has_pan_control() || mount->mount_yaw_follow_mode == AP_Mount::gimbal_yaw_follows_vehicle)
        {
          //in this case, APM is getting yaw rate commands from the tablet via mavlink,
          //so we set the set the fixed yaw command to this rate, and tell the
          //angle controller in mode guided to interpret as a rate. Note that the
          //auto yaw will be set to mode fixed, though it is actually controlling rate.
          is_payload_yaw_rate = true;

          if(copter.mode_guided.mode()==Guided_Angle)
          {
            //If we've stopped getting the mavlink yaw rate commands, use 0 yaw rate command
            if((AP_HAL::micros64() - mount->get_last_mount_control_time_us()) > 50000)
            {
              copter.flightmode->auto_yaw.set_fixed_yaw(
                    0.0f,
                    0.0f,
                    0,
                    0);
            }
            else if(copter.flightmode->auto_yaw.mode() == AUTO_YAW_RATE)
            {
              copter.flightmode->auto_yaw.set_fixed_yaw(
                    copter.flightmode->auto_yaw.rate_cds(),
                    0.0f,
                    0,
                    0);

            }
          }
          else
          {
            //if we have tablet rate commands have been received and are not stale, and we're in posvel,
            //then use tablet (payload) yaw rate commands instead of ACE yaw/yaw rate commands
            if((copter.flightmode->auto_yaw.mode() == AUTO_YAW_RATE) && ((AP_HAL::micros64() - mount->get_last_mount_control_time_us()) <= 500000))
            {
               payload_yaw_rate_cds = copter.flightmode->auto_yaw.rate_cds();
               use_payload_yaw_rate = true;
            }
          }
        }
      }
    }

    if(high_tension) {
        //If we ever enter high tension, make sure ACE is landing
        if((copter.flightmode != &copter.mode_planckland) && (copter.flightmode != &copter.mode_planckrtb) && copter.planck_interface.ready_for_land()) {
            copter.set_mode_planck_RTB_or_planck_land(ModeReason::GCS_FAILSAFE);
        }

        if(!copter.planck_interface.check_for_high_tension_timeout()) { //High tension hasn't failed
            //While in high tension:
            // - If actively tracking the tag, continue to do so, but use pos throttle
            // - If not tracking the tag, but have GPS, command zero velocity but use pos throttle
            // - If not tracking tag or and no GPS, use zero attitude with pos throttle
            if(!copter.planck_interface.get_tag_tracking_state()) {
                if(copter.position_ok()) {
                  copter.planck_interface.override_with_zero_vel_cmd();

                  //Force the position controller to use the current position
                  const Vector3f& curr_pos = inertial_nav.get_position();
                  // set target position to current position
                  pos_control->set_xy_target(curr_pos.x, curr_pos.y);
                } else {
                  copter.planck_interface.override_with_zero_att_cmd();
                }
            }
        } else { //High tension has timed out
            //If high tension has failed, attempt to use a planck land or regular land
            copter.set_mode_land_with_pause(ModeReason::GCS_FAILSAFE, false);
            copter.mode_land.run();
            return;
        }
    }

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

              float min_yaw_alt_cm = (mount != nullptr ? mount->get_min_yaw_alt_cm() : 1000.f);

              //If we are in WINGMAN mode, the user controls yaw, even though
              //we might be getting attitude/acceleration commands
              if(copter.flightmode == &copter.mode_planckwingman && !copter.failsafe.radio)
              {
                  // get pilot's desired yaw rate
                  float pilot_yaw_rate_cds = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
                  yaw_cd = pilot_yaw_rate_cds;
                  is_yaw_rate = true;
              }
              //Some applications require a fixed yaw command when in PLANCKTRACK
              //Only use gimbal yaw (or yaw rate) commands if in mode_plancktracking and above alt threshold, and not high tension
              else if((copter.flightmode == &copter.mode_plancktracking)
                      && (copter.flightmode->auto_yaw.mode() == AUTO_YAW_FIXED )
                      && (copter.planck_interface.get_tag_pos().z > min_yaw_alt_cm)
                      && !high_tension)
              {
                  yaw_cd = copter.flightmode->auto_yaw.yaw();
                  is_yaw_rate = is_payload_yaw_rate;
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
              bool is_yaw_rate;
              float yaw_rate_cmd = 0;

              bool good_cmd = copter.planck_interface.get_posvel_cmd(
                loc_cmd,
                vel_cmd,
                yaw_cmd,
                is_yaw_rate);

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

                  //use payload yaw rate if we have it
                  if(use_payload_yaw_rate && is_payload_yaw_rate)
                  {
                    yaw_rate_cmd = payload_yaw_rate_cds;
                    is_yaw_rate = use_payload_yaw_rate;
                  }
                  else if(is_yaw_rate)
                  {
                    yaw_rate_cmd = yaw_cmd;
                  }
                  ModeGuided::set_destination_posvel(pos_cmd,vel_cmd,!is_yaw_rate,yaw_cmd,is_yaw_rate,yaw_rate_cmd);
              }
              break;
          }

          default:
            break;
      }
    }

    //Run the guided mode controller
    ModeGuided::run(true, use_positive_throttle); //use high-jerk, positive throttle if necessary
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

void ModePlanckTracking::exit()
{
  auto_yaw.set_mode_to_default(false);
}
