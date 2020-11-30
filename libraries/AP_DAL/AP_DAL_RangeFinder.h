#pragma once

#include <AP_RangeFinder/AP_RangeFinder.h>

#include <AP_Logger/LogStructure.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_RangeFinder_Backend;

class AP_DAL_RangeFinder {
public:

    // RangeFinder-like methods:
    enum class Status {
        NotConnected = 0,
        NoData,
        OutOfRangeLow,
        OutOfRangeHigh,
        Good
    };

    int16_t ground_clearance_cm_orient(enum Rotation orientation) const;
    int16_t max_distance_cm_orient(enum Rotation orientation) const;

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // DAL methods:
    AP_DAL_RangeFinder();

    void start_frame();

    class AP_DAL_RangeFinder_Backend *get_backend(uint8_t id) const;

    void handle_message(const log_RRNH &msg);
    void handle_message(const log_RRNI &msg);

private:

    struct log_RRNH _RRNH;
    struct log_RRNI *_RRNI;
    AP_DAL_RangeFinder_Backend **_backend;
};


class AP_DAL_RangeFinder_Backend {
public:

    AP_DAL_RangeFinder_Backend(struct log_RRNI &RRNI);

    // RangeFinder-backend-like methods

    enum Rotation orientation() const {
        return (Rotation)_RRNI.orientation;
    }

    AP_DAL_RangeFinder::Status status() const {
        return (AP_DAL_RangeFinder::Status)_RRNI.status;
    }

    uint16_t distance_cm() const { return _RRNI.distance_cm; }

    const Vector3f &get_pos_offset() const { return _RRNI.pos_offset; }

    // DAL methods:
    void start_frame(AP_RangeFinder_Backend *backend);

private:

    struct log_RRNI &_RRNI;
};
