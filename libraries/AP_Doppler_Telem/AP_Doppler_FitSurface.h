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

#include <AP_Math/AP_Math.h>
#include "AP_Doppler_Telem.h"

#define AP_DOPPLER_FITSURFACE_DISTANCEERROR 0.05f // 距离误差，单位 m

class AP_Doppler_FitSurface
{
public:
    struct BeamGeometry {
        float gamma_rad;   // 与传感器主轴夹角
        float beta_rad;    // 围绕主轴的方位角
    };

    AP_Doppler_FitSurface(AP_Doppler_Telem& doppler,
                          const BeamGeometry& beam_a,
                          const BeamGeometry& beam_b,
                          const BeamGeometry& beam_c,
                          const BeamGeometry& beam_d,
                          float max_residual_m,
                          enum Rotation sensor_to_body_rot = ROTATION_NONE);

    // 读取4束DVL并更新平面拟合结果
    bool update();

    // 结果接口
    const Vector3f& get_plane_normal_body()   const { return _normal_body;   }
    float get_plane_distance_m() const { return fabs(_plane_d_sensor); }

private:
    struct BeamSample {
        bool valid = false;
        float distance_m = 0.0f;
        float gamma_rad = 0.0f;
        float beta_rad = 0.0f;
        float cos_gamma = 1.0f;
        float sin_gamma = 0.0f;
        float cos_beta  = 1.0f;
        float sin_beta  = 0.0f;
        Vector3f vec_sensor;

        void init(float gamma, float beta);
        void update_from_distance(float distance_m_in, bool valid_in);
    };

    bool read_beams();
    bool fit_plane_from_valid_points();
    bool fit_plane_3points(const Vector3f* pts, uint8_t count);
    bool fit_plane_4points_ls(const Vector3f* pts);
    void update_body_frame_outputs();
    void reset_solution();

    // 3x3 对称矩阵最小特征向量（Jacobi）
    static bool smallest_eigenvector_sym_3x3(const float A_in[3][3], Vector3f& eigvec);

    AP_Doppler_Telem& _doppler;
    enum Rotation _sensor_to_body_rot;
    float _max_residual_m;

    BeamSample _beam_a;
    BeamSample _beam_b;
    BeamSample _beam_c;
    BeamSample _beam_d;

    bool _healthy = false;
    uint8_t _beam_count = 0;

    Vector3f _normal_sensor;
    Vector3f _normal_body;
    float _plane_d_sensor = 0.0f;
    float _plane_d_last_sensor = 0.0f;
    float _residual_m = 0.0f;
};
