#include "AP_DAL_Airspeed.h"
#include "AP_DAL.h"

#include <AP_Logger/AP_Logger.h>

AP_DAL_Airspeed::AP_DAL_Airspeed()
{
#if AP_AIRSPEED_ENABLED
    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        _RASI[i].instance = i;
    }
#endif
}

void AP_DAL_Airspeed::start_frame()
{
#if AP_AIRSPEED_ENABLED
    const auto *airspeed = AP::airspeed();
    if (airspeed == nullptr) {
        return;
    }

    const log_RASH old = _RASH;
    _RASH.num_sensors = airspeed->get_num_sensors();
    _RASH.primary = airspeed->get_primary();
    WRITE_REPLAY_BLOCK_IFCHANGED(RASH, _RASH, old);

    for (uint8_t i=0; i<ARRAY_SIZE(_RASI); i++) {
        log_RASI &RASI = _RASI[i];
        log_RASI old_RASI = RASI;
        RASI.last_update_ms = airspeed->last_update_ms(i);
        RASI.healthy = airspeed->healthy(i);
        RASI.use = airspeed->use(i);
        RASI.airspeed = airspeed->get_airspeed(i);
        WRITE_REPLAY_BLOCK_IFCHANGED(RASI, RASI, old_RASI);
    }
#endif
}
