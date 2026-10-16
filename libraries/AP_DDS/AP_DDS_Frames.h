#pragma once

#include <AP_Math/rotations.h>
#include <AP_Math/quaternion.h>

static constexpr char WGS_84_FRAME_ID[] = "WGS-84";
// https://www.ros.org/reps/rep-0105.html#base-link
static constexpr char BASE_LINK_FRAME_ID[] = "base_link";
static constexpr char BASE_LINK_NED_FRAME_ID[] = "base_link_ned";
// https://www.ros.org/reps/rep-0105.html#map
static constexpr char MAP_FRAME[] = "map";

// Z-axis 90° rotation: (w=√2/2, x=0, y=0, z=√2/2), ROS REP-103 NED→ENU
static const Quaternion FRAME_Z_ROTATION_90 {HALF_SQRT_2, 0.0f, 0.0f, HALF_SQRT_2};
