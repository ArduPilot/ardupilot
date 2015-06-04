// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  additional arming checks for plane
 */
#include "Plane.h"

/*
  additional arming checks for plane
 */
bool AP_Arming_Plane::pre_arm_checks(bool report)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(report);

    if (plane.g.roll_limit_cd < 300) {
        if (report) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: LIM_ROLL_CD too small"));        
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        if (report) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: LIM_PITCH_MAX too small"));        
        }
        ret = false;        
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        if (report) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: LIM_PITCH_MIN too large"));        
        }
        ret = false;        
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->radio_max) {
        if (report) {
            gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: invalid THR_FS_VALUE for rev throttle"));        
        }
        ret = false;
    }

    return ret;
}
