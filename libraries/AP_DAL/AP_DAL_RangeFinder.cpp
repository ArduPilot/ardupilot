#include "AP_DAL_RangeFinder.h"

#include <AP_Logger/AP_Logger.h>

#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include "AP_DAL.h"

AP_DAL_RangeFinder::AP_DAL_RangeFinder()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RRNI); i++) {
        _RRNI[i].instance = i;
    }

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // this avoids having to discard a const....
        _backend[i] = new AP_DAL_RangeFinder_Backend(_RRNI[i]);
    }
}

int16_t AP_DAL_RangeFinder::ground_clearance_cm_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    const auto *rangefinder = AP::rangefinder();

    if (orientation != ROTATION_PITCH_270) {
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->ground_clearance_cm_orient(orientation);
    }
#endif

    return _RRNH.ground_clearance_cm;
}

int16_t AP_DAL_RangeFinder::max_distance_cm_orient(enum Rotation orientation) const
{
#if !APM_BUILD_TYPE(APM_BUILD_AP_DAL_Standalone)
    if (orientation != ROTATION_PITCH_270) {
        const auto *rangefinder = AP::rangefinder();
        // the EKF only asks for this from a specific orientation.  Thankfully.
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return rangefinder->ground_clearance_cm_orient(orientation);
    }
#endif

    return _RRNH.max_distance_cm;
}

void AP_DAL_RangeFinder::start_frame()
{
    const auto *rangefinder = AP::rangefinder();
    if (rangefinder == nullptr) {
        return;
    }

    const log_RRNH old = _RRNH;

    // EKF only asks for this *down*.
    _RRNH.ground_clearance_cm = rangefinder->ground_clearance_cm_orient(ROTATION_PITCH_270);
    _RRNH.max_distance_cm = rangefinder->max_distance_cm_orient(ROTATION_PITCH_270);
    _RRNH.backend_mask = 0;

    for (uint8_t i=0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        auto *backend = rangefinder->get_backend(i);
        if (backend == nullptr) {
            break;
        }
        _RRNH.backend_mask |= (1U<<i);
        _backend[i]->start_frame(backend);
    }

    WRITE_REPLAY_BLOCK_IFCHANGED(RRNH, _RRNH, old);
}




AP_DAL_RangeFinder_Backend::AP_DAL_RangeFinder_Backend(struct log_RRNI &RRNI) :
    _RRNI(RRNI)
{
}

void AP_DAL_RangeFinder_Backend::start_frame(AP_RangeFinder_Backend *backend) {
    const log_RRNI old = _RRNI;
    _RRNI.orientation = backend->orientation();
    _RRNI.status = (uint8_t)backend->status();
    _RRNI.pos_offset = backend->get_pos_offset();
    _RRNI.distance_cm = backend->distance_cm();
    WRITE_REPLAY_BLOCK_IFCHANGED(RRNI, _RRNI, old);
}


AP_DAL_RangeFinder_Backend *AP_DAL_RangeFinder::get_backend(uint8_t id) const
{
   if (id > RANGEFINDER_MAX_INSTANCES) {
        INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
        return nullptr;
    }

   if ((_RRNH.backend_mask & (1U<<id)) == 0) {
       return nullptr;
   }

   return _backend[id];
}
