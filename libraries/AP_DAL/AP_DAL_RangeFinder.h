#pragma once

#include <AP_RangeFinder/AP_RangeFinder.h>

#if AP_RANGEFINDER_ENABLED

#include <AP_Logger/LogStructure.h>

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

    float ground_clearance_orient(enum Rotation orientation) const;
    float max_distance_orient(enum Rotation orientation) const;

    // return true if we have a range finder with the specified orientation
    bool has_orientation(enum Rotation orientation) const;

    // true when a backend's reading time is known: always in flight, and in Replay once the log gives one
    bool have_reading_time(uint8_t instance) const {
        return instance < ARRAY_SIZE(_have_reading_time) && _have_reading_time[instance];
    }

    // DAL methods:
    AP_DAL_RangeFinder();

    void start_frame();

    class AP_DAL_RangeFinder_Backend *get_backend(uint8_t id) const;

    void handle_message(const log_RRNH &msg);
    void handle_message(const log_RRNI &msg);
    void handle_message(const log_RRNJ &msg);

private:

    struct log_RRNH _RRNH;
    struct log_RRNI *_RRNI;
    struct log_RRNJ *_RRNJ;
    bool _have_reading_time[RANGEFINDER_MAX_INSTANCES];
    AP_DAL_RangeFinder_Backend **_backend;
};


class AP_DAL_RangeFinder_Backend {
public:

    AP_DAL_RangeFinder_Backend(struct log_RRNI &RRNI, struct log_RRNJ &RRNJ);

    // RangeFinder-backend-like methods

    enum Rotation orientation() const {
        return (Rotation)_RRNI.orientation;
    }

    AP_DAL_RangeFinder::Status status() const {
        return (AP_DAL_RangeFinder::Status)_RRNI.status;
    }

    float distance() const { return _RRNI.distance; }

    const Vector3f &get_pos_offset() const { return _RRNI.pos_offset; }

    // system time of the backend's last reading, if AP_DAL_RangeFinder::have_reading_time()
    uint32_t last_reading_ms() const { return _RRNJ.last_reading_ms; }

    // DAL methods:
    void start_frame(AP_RangeFinder_Backend *backend);

private:

    struct log_RRNI &_RRNI;
    struct log_RRNJ &_RRNJ;
};

#endif  // AP_RANGEFINDER_ENABLED
