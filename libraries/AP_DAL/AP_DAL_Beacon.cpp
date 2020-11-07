#include "AP_DAL_Beacon.h"

#include <AP_Beacon/AP_Beacon.h>

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_Beacon::AP_DAL_Beacon()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        _RBCI[i].instance = i;
    }
}

void AP_DAL_Beacon::start_frame()
{
    const auto *bcon = AP::beacon();

    const log_RBCH old = _RBCH;
    _RBCH.ptr_is_nullptr = (bcon == nullptr);
    if (bcon != nullptr) {
        _RBCH.count = bcon->count();
        _RBCH.get_vehicle_position_ned_returncode = bcon->get_vehicle_position_ned(_RBCH.vehicle_position_ned, _RBCH.accuracy_estimate);

        Location loc;
        _RBCH.get_origin_returncode = bcon->get_origin(loc);
        _RBCH.origin_lat = loc.lat;
        _RBCH.origin_lng = loc.lng;
        _RBCH.origin_alt = loc.alt;
    }
    WRITE_REPLAY_BLOCK_IFCHANGED(RBCH, _RBCH, old);
    if (bcon == nullptr) {
        return;
    }

    for (uint8_t i=0; i<ARRAY_SIZE(_RBCI); i++) {
        log_RBCI &RBCI = _RBCI[i];
        const log_RBCI old_RBCI = RBCI;
        RBCI.last_update_ms = bcon->beacon_last_update_ms(i);
        RBCI.position = bcon->beacon_position(i);
        RBCI.distance = bcon->beacon_distance(i);
        RBCI.healthy = bcon->beacon_healthy(i);

        WRITE_REPLAY_BLOCK_IFCHANGED(RBCI, RBCI, old_RBCI);
    }
}
