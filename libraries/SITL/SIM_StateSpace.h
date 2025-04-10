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
  State Space simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include <AP_JSON/AP_JSON.h>


namespace SITL {

/*
  a State Space dynamics model simulator
 */
class StateSpace : public Aircraft {
public:
    StateSpace(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW StateSpace(frame_str);
    }

protected:

enum frame_types {
    HELI_FRAME,
    MULTI_FRAME,
} frame_type = MULTI_FRAME;

    void update_rotor_dynamics(Vector3f gyros, Vector2f ctrl_pos, Vector2f &tpp_angle, float dt);
    float update_rpm(float curr_rpm, float throttle, float nom_rpm, float dt);
    void Log_Write_SimData(float dlong, float dlat, float dthr, float dped);

    struct servos_stored {
        float servo1;
        float servo2;
    };
    float _servos_delayed_rp[2];
    ObjectBuffer<servos_stored> *servos_stored_buffer;
    void push_to_buffer_rp(const float servos_input[16]);
    void pull_from_buffer_rp(float servos_delayed_rp[2]);

    struct servo_stored {
        float servo;
    };
    // PED
    float _servo_delayed_ped;
    ObjectBuffer<servo_stored> *servo_stored_ped_buffer;
    void push_to_buffer_ped(const float servos_input[16]);
    void pull_from_buffer_ped(float &servo_delayed);

    // COL
    float _servo_delayed_col;
    ObjectBuffer<servo_stored> *servo_stored_col_buffer;
    void push_to_buffer_col(const float servos_input[16]);
    void pull_from_buffer_col(float &servo_delayed);

    void load_frame_params(const char *model_json);
    void parse_float(AP_JSON::value val, const char* label, float &param);

    const struct Model {

        float Ma1s = 0;
        float Lb1s = 0;
        float Xa1s = 0;
        float Yb1s = 0;
        float Mu = 0;
        float Mv = 0;
        float Mq = 0;
        float Mp = 0;
        float Lu = 0;
        float Lv = 0;
        float Lp = 0;
        float Lq = 0;
        float Lr = 0;
        float Xu = 0;
        float Yv = 0;
        float Yr = 0;
        float Yp = 0;
        float Zw = 0;
        float Nr = 0;
        float Nw = 0;
        float Nv = 0;
        float Ncol = 0;
        float Mlon = 0;
        float Mlat = 0;
        float Llon = 0;
        float Llat = 0;
        float Nped = 0;
        float Xlon = 0;
        float Ylat = 0;
        float Zcol = 0;
        float time_delay_rp = 0;
        float time_delay_ped = 0;
        float time_delay_col = 0;
        float Lag = 0;
        float Lead = 0;
        float tf = 0;
        float Lfa1s = 0;
        float Mfb1s = 0;
        float Lflt = 0;
        float Lflg = 0;
        float Mflt = 0;
        float Mflg = 0;
        float hover_lean = 0;
        float nominal_rpm = 0;
        float coll_max = 0;
        float coll_min = 0;
        float coll_hover = 0;
        float thr_hover = 0;

    } default_model;

    struct Model model;

private:
    // input data
    float hover_coll = 0.5f;


    // variables
    Vector2f _tpp_angle;

};

} // namespace SITL
