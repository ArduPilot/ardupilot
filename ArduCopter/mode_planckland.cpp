#include "Copter.h"

bool Copter::ModePlanckLand::init(bool ignore_checks){

    //If we are already landed this makes no sense
    if(copter.ap.land_complete)
      return false;

    if(!copter.planck_interface.ready_for_land()) {
      return false;
    }

    Copter::ModeGuided::set_angle(Quaternion(),0,true,0);
    if(Copter::ModeGuidedNoGPS::init(ignore_checks)) {
        float land_velocity = abs((copter.g.land_speed > 0 ?
            copter.g.land_speed : copter.pos_control->get_speed_down()))/100.;
      copter.planck_interface.request_land(land_velocity);
      return true;
    }
    return false;
}

void Copter::ModePlanckLand::run(){
    copter.mode_plancktracking.run();
}
