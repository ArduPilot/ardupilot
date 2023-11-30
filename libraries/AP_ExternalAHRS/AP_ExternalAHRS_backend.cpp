/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  parent class for ExternalAHRS backends
 */

#include "AP_ExternalAHRS_backend.h"
#include <AP_AHRS/AP_AHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS.h>

AP_ExternalAHRS_backend::AP_ExternalAHRS_backend(AP_ExternalAHRS *_frontend,
                                                 AP_ExternalAHRS::state_t &_state) :
    frontend(*_frontend),
    state(_state)
{}


uint16_t AP_ExternalAHRS_backend::get_rate(void) const
{
    return frontend.get_IMU_rate();
}

bool AP_ExternalAHRS_backend::option_is_set(AP_ExternalAHRS::OPTIONS option) const
{
    return frontend.option_is_set(option);
}

bool AP_ExternalAHRS_backend::in_fly_forward(void) const
{
    return AP::ahrs().get_fly_forward();
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_backend::send_EKF_status_report(class GCS_MAVLINK &link)
{
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus {};
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initialized) {
        flags |= EKF_UNINITIALIZED;
    }
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       state.velocity_variance,
                                       state.pos_horiz_variance,
                                       state.pos_vert_variance,
                                       state.compass_variance,
                                       state.terrain_alt_variance,
                                       state.airspeed_variance);
}

#endif  // HAL_EXTERNAL_AHRS_ENABLED

