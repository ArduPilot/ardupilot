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
#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Doppler_Backend.h"
#include "AP_Doppler_FitSurface.h"

class AP_Doppler_Parameters;

class AP_Doppler_Telem {

public:
    AP_Doppler_Telem();

    ~AP_Doppler_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Doppler_Telem);

    // init - perform required initialisation
    bool init(const AP_SerialManager &serial_manager);
    void update();
    void send();
    bool get_velocity_body(Vector3f &vel_body_mps, uint32_t &t_ms, float &quality, DVL_LockState &lock) const;
    bool get_bi_msg(DVL_BI_Msg &msg) const;
    bool get_bd_msg(DVL_BD_Msg &msg) const;
    bool get_wi_msg(DVL_WI_Msg &msg) const;
    bool get_ua_msg(DVL_U_Msg &msg) const;
    bool get_ub_msg(DVL_U_Msg &msg) const;
    bool get_uc_msg(DVL_U_Msg &msg) const;
    bool get_ud_msg(DVL_U_Msg &msg) const;
    const AP_Doppler_Parameters &parameters() const { return *_doppler_parameters; }

    static AP_Doppler_Telem *get_singleton(void) {
        return singleton;
    }

    Vector3f get_normal_body()   const { if(!fit_health) return Vector3f(0,0,0); return _fitsurface->get_normal_body(); }
    float get_distance_m() const { if(!fit_health) return 0; return _fitsurface->get_distance_m(); }
    float get_z_vel_mps() const { Vector3f vel_mps {}; uint32_t t; float q; DVL_LockState l ; get_velocity_body(vel_mps, t, q, l); return vel_mps.z; }

    void set_sensor_to_body_rot(enum Rotation rot) { _fitsurface->set_sensor_to_body_rot(rot); }

protected:
    

private:
    void update_simulated_messages();

    AP_Doppler_Parameters *_doppler_parameters;
    AP_HAL::UARTDriver *port;
    AP_Doppler_Backend *_backend;
    AP_Doppler_FitSurface *_fitsurface;
    mutable HAL_Semaphore _sim_sem;
    uint32_t _sim_last_update_ms = 0;
    DVL_BI_Msg _sim_bi_msg {};
    DVL_BD_Msg _sim_bd_msg {};
    DVL_WI_Msg _sim_wi_msg {};
    DVL_U_Msg _sim_ua_msg {};
    DVL_U_Msg _sim_ub_msg {};
    DVL_U_Msg _sim_uc_msg {};
    DVL_U_Msg _sim_ud_msg {};
    static AP_Doppler_Telem *singleton;
    bool fit_health = false;

};

namespace AP {
    AP_Doppler_Telem *Doppler_telem();
};

