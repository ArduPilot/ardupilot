/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Siddharth Bharat Purohit
 */

#pragma once

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_DIGITAL_SKY_RFM

#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>


#ifndef AP_NPNT_PERMART_FILE
#define AP_NPNT_PERMART_FILE HAL_BOARD_STORAGE_DIRECTORY "/permissionArtifact.xml"
#endif


class AP_DSNPNT {

public:
    AP_DSNPNT();

    // get singleton instance
    static AP_DSNPNT *get_singleton() {
        return _singleton;
    }
    bool load_permission();
    bool update_permission(bool do_log, Vector2f &curr_loc, time_t &t_now, float &alt);
    void verify_permission();
    void handle_set_dsnpnt_params(const mavlink_message_t &msg);
    void handle_get_dsnpnt_params(mavlink_channel_t chan, const mavlink_message_t &msg);
private:
    bool _check_npnt_permission();
    static AP_DSNPNT *_singleton;
    bool permission_granted;
    bool verify_permission_reg;
    uint32_t last_verify_time_ms;
    Vector2f *fence_verts;
};

namespace AP {
    AP_DSNPNT &dsnpnt();
};

#endif //#ifdef HAL_DIGITAL_SKY_RFM
