#pragma once

// Camera related definitions required by both AP_Camera and AP_Mount are here
// this avoids issues that would occur if AP_Mount and AP_Camera included each other

#include <AP_Math/AP_Math.h>

// set zoom specified as a rate or percentage
// enumerators match MAVLink CAMERA_ZOOM_TYPE
enum class ZoomType : uint8_t {
    RATE = 1,   // zoom in, out or hold (zoom out = -1, hold = 0, zoom in = 1). Same as ZOOM_TYPE_CONTINUOUS
    PCT = 2     // zoom to a percentage (from 0 to 100) of the full range. Same as ZOOM_TYPE_RANGE
};

