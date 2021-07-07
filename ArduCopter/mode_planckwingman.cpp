#include "Copter.h"

bool ModePlanckWingman::init(bool ignore_checks){

    //Make sure we don't immediately send a different command
    _next_req_send_t_ms = millis() + _send_rate_ms;

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;

    //Otherwise, run tracking
    if(copter.mode_plancktracking.init_without_RTB_request(ignore_checks)) {

      //And command a zero-rate, telling planck to maintain current relative position
      copter.planck_interface.request_move_target(Vector3f(0,0,0),true);
      return true;
    }

    return false;
}

void ModePlanckWingman::run() {

  //Update the command if the user is providing input and we're not in AUTO
  if( !copter.failsafe.radio && copter.flightmode != &copter.mode_auto) {

    //Rate limited
    if(millis() > _next_req_send_t_ms) {

      //Get position/yaw offsets from user as necessary
      float x_rate = -channel_pitch->norm_input_dz() * copter.wp_nav->get_default_speed_xy()/100.;
      float y_rate = channel_roll->norm_input_dz() * copter.wp_nav->get_default_speed_xy()/100.;

      //Turn x/y rates into N/E rates
      float N_rate = (x_rate*copter.ahrs.cos_yaw() - y_rate*copter.ahrs.sin_yaw());
      float E_rate = (x_rate*copter.ahrs.sin_yaw() + y_rate*copter.ahrs.cos_yaw());
      float z_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in())/100.;

      Vector3f rate_NED(N_rate, E_rate, z_rate); //NED

      copter.planck_interface.request_move_target(rate_NED, true);

      _next_req_send_t_ms = millis() + _send_rate_ms;
    }
  }

  copter.mode_plancktracking.run();
}

void ModePlanckWingman::exit()
{
  auto_yaw.set_mode_to_default(false);
}
