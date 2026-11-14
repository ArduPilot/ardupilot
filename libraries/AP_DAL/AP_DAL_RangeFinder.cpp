#include "AP_DAL_RangeFinder.h"

#include <AP_Logger/AP_Logger.h>

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>

#if AP_RANGEFINDER_ENABLED

#include "AP_DAL.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_InternalError/AP_InternalError.h>

AP_DAL_RangeFinder::AP_DAL_RangeFinder()
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone) && !APM_BUILD_TYPE(APM_BUILD_Replay)
    _RRNH.num_sensors = AP::rangefinder()->num_sensors();
    _RRNI = NEW_NOTHROW log_RRNI[_RRNH.num_sensors];
    _RRNJ = NEW_NOTHROW log_RRNJ[_RRNH.num_sensors];
    _backend = NEW_NOTHROW AP_DAL_RangeFinder_Backend *[_RRNH.num_sensors];
    if (!_RRNI || !_RRNJ || !_backend) {
        goto failed;
    }
    for (uint8_t i=0; i<_RRNH.num_sensors; i++) {
        _RRNI[i].instance = i;
        _RRNJ[i].instance = i;
        _have_reading_time[i] = true;
    }
    for (uint8_t i=0; i<_RRNH.num_sensors; i++) {
        // this avoids having to discard a const....
        _backend[i] = NEW_NOTHROW AP_DAL_RangeFinder_Backend(_RRNI[i], _RRNJ[i]);
        if (!_backend[i]) {
            goto failed;
        }
    }
    return;
failed:
    AP_BoardConfig::allocation_error("DAL backends");
#endif
}

float AP_DAL_RangeFinder::ground_clearance_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    const auto *rangefinder = AP::rangefinder();

    if (orientation != ROTATION_PITCH_270) {
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->ground_clearance_orient(orientation);
    }
#endif

    return _RRNH.ground_clearance;
}

float AP_DAL_RangeFinder::max_distance_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    if (orientation != ROTATION_PITCH_270) {
        const auto *rangefinder = AP::rangefinder();
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->max_distance_orient(orientation);
    }
#endif

    return _RRNH.max_distance;
}

void AP_DAL_RangeFinder::start_frame()
{
    const auto *rangefinder = AP::rangefinder();
    if (rangefinder == nullptr) {
        return;
    }

    const log_RRNH old = _RRNH;

    // EKF only asks for this *down*.
    _RRNH.ground_clearance = rangefinder->ground_clearance_orient(ROTATION_PITCH_270);
    _RRNH.max_distance = rangefinder->max_distance_orient(ROTATION_PITCH_270);

    WRITE_REPLAY_BLOCK_IFCHANGED(RRNH, _RRNH, old);

    for (uint8_t i=0; i<_RRNH.num_sensors; i++) {
        auto *backend = rangefinder->get_backend(i);
        if (backend == nullptr) {
            continue;
        }
        _backend[i]->start_frame(backend);
    }
}




AP_DAL_RangeFinder_Backend::AP_DAL_RangeFinder_Backend(struct log_RRNI &RRNI, struct log_RRNJ &RRNJ) :
    _RRNI(RRNI),
    _RRNJ(RRNJ)
{
}

void AP_DAL_RangeFinder_Backend::start_frame(AP_RangeFinder_Backend *backend) {
    const log_RRNI old = _RRNI;
    _RRNI.orientation = backend->orientation();
    _RRNI.status = (uint8_t)backend->status();
    _RRNI.pos_offset = backend->get_pos_offset();
    _RRNI.distance = backend->distance();
    WRITE_REPLAY_BLOCK_IFCHANGED(RRNI, _RRNI, old);

    const log_RRNJ old_RRNJ = _RRNJ;
    _RRNJ.last_reading_ms = backend->last_reading_ms();
    WRITE_REPLAY_BLOCK_IFCHANGED(RRNJ, _RRNJ, old_RRNJ);
}

// return true if we have a range finder with the specified orientation
bool AP_DAL_RangeFinder::has_orientation(enum Rotation orientation) const
{
    for (uint8_t i=0; i<_RRNH.num_sensors; i++) {
        if (_RRNI[i].orientation == orientation) {
            return true;
        }
    }
    return false;
}


AP_DAL_RangeFinder_Backend *AP_DAL_RangeFinder::get_backend(uint8_t id) const
{
   if (id >= RANGEFINDER_MAX_INSTANCES) {
       INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
       return nullptr;
   }
   if (id >= _RRNH.num_sensors) {
        return nullptr;
    }

   return _backend[id];
}

void AP_DAL_RangeFinder::handle_message(const log_RRNH &msg)
{
    _RRNH = msg;
    if (_RRNH.num_sensors > 0 && _RRNI == nullptr) {
        _RRNI = NEW_NOTHROW log_RRNI[_RRNH.num_sensors];
        _RRNJ = NEW_NOTHROW log_RRNJ[_RRNH.num_sensors];
        _backend = NEW_NOTHROW AP_DAL_RangeFinder_Backend *[_RRNH.num_sensors];
    }
}

void AP_DAL_RangeFinder::handle_message(const log_RRNI &msg)
{
    if (_RRNI != nullptr && _RRNJ != nullptr && msg.instance < _RRNH.num_sensors) {
        _RRNI[msg.instance] = msg;
        if (_backend != nullptr && _backend[msg.instance] == nullptr) {
            _backend[msg.instance] = NEW_NOTHROW AP_DAL_RangeFinder_Backend(_RRNI[msg.instance], _RRNJ[msg.instance]);
        }
    }
}

void AP_DAL_RangeFinder::handle_message(const log_RRNJ &msg)
{
    if (_RRNJ != nullptr && msg.instance < _RRNH.num_sensors) {
        _RRNJ[msg.instance] = msg;
        // RRNJ is written at the start of a log, so this comes before the EKF first runs; a log from
        // before RRNJ never gives one
        if (msg.instance < ARRAY_SIZE(_have_reading_time)) {
            _have_reading_time[msg.instance] = true;
        }
    }
}

#endif  // AP_RANGEFINDER_ENABLED
