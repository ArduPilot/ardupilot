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
 */
#pragma once

#include "AP_CustomRotations_config.h"

#if AP_CUSTOMROTATIONS_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define NUM_CUST_ROT ROTATION_CUSTOM_END - ROTATION_CUSTOM_1

struct AP_CustomRotation_params {
public:
    AP_CustomRotation_params();

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float roll;
    AP_Float pitch;
    AP_Float yaw;
};

class AP_CustomRotation {
public:
    AP_CustomRotation(AP_CustomRotation_params &_params);

    void init();

    Quaternion q;
    Matrix3f m;

    AP_CustomRotation_params &params;
};

class AP_CustomRotations {
public:
    AP_CustomRotations();

    CLASS_NO_COPY(AP_CustomRotations);

    static AP_CustomRotations *get_singleton(void) { return singleton; }

    void init();

    void from_rotation(enum Rotation r, QuaternionD &q);
    void from_rotation(enum Rotation r, Quaternion &q);

    void rotate(enum Rotation r, Vector3d& v);
    void rotate(enum Rotation r, Vector3f& v);

    void convert(Rotation r, float roll, float pitch, float yaw);
    void set(Rotation r, float roll, float pitch, float yaw);

    static const struct AP_Param::GroupInfo var_info[];

private:

    AP_Int8 enable;

    AP_CustomRotation* get_rotation(Rotation r);

    AP_CustomRotation* rotations[NUM_CUST_ROT];

    AP_CustomRotation_params params[NUM_CUST_ROT];

    static AP_CustomRotations *singleton;
};

namespace AP {
    AP_CustomRotations &custom_rotations();
};


#endif  // AP_CUSTOMROTATIONS_ENABLED
