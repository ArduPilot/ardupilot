#pragma once

// Camera related definitions required by both AP_Camera and AP_Mount are here
// this avoids issues that would occur if AP_Mount and AP_Camera included each other

#include <stdint.h>

// set zoom specified as a rate or percentage
// enumerators match MAVLink CAMERA_ZOOM_TYPE
enum class ZoomType : uint8_t {
    RATE = 1,   // zoom in, out or hold (zoom out = -1, hold = 0, zoom in = 1). Same as ZOOM_TYPE_CONTINUOUS
    PCT = 2     // zoom to a percentage (from 0 to 100) of the full range. Same as ZOOM_TYPE_RANGE
};

// set focus specified as a rate or percentage
// enumerators match MAVLink CAMERA_FOCUS_TYPE
enum class FocusType : uint8_t {
    RATE = 1,   // focus in, out or hold (focus in = -1, hold = 0, focus out = 1). Same as FOCUS_TYPE_CONTINUOUS
    PCT = 2,    // focus to a percentage (from 0 to 100) of the full range. Same as FOCUS_TYPE_RANGE
    AUTO = 4    // focus automatically. Same as FOCUS_TYPE_AUTO
};

// result type of set_focus.  Assumptions are made that this
// enumeration can be cast directly to MAV_RESULT.
enum class SetFocusResult : uint8_t {
    ACCEPTED = 0,
    INVALID_PARAMETERS = 2,  // supported but invalid parameters, like MAV_RESULT_DENIED
    UNSUPPORTED = 3,
    FAILED = 4,
};

// tracking types when tracking an object in the video stream
enum class TrackingType : uint8_t {
    TRK_NONE = 0,       // tracking is inactive
    TRK_POINT = 1,      // tracking a point
    TRK_RECTANGLE = 2   // tracking a rectangle
};

// camera settings not normally used by the autopilot
enum class CameraSetting {
    THERMAL_PALETTE = 0,    // set thermal palette
    THERMAL_GAIN = 1,       // set thermal gain, value of 0:low gain, 1:high gain
    THERMAL_RAW_DATA = 2,   // enable/disable thermal raw data
};
