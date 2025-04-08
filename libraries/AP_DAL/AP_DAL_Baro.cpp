#include "AP_DAL_Baro.h"

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_Baro::AP_DAL_Baro()
{
    for (uint8_t i=0; i<BARO_MAX_INSTANCES; i++) {
        _RBRI[i].instance = i;
    }
}

void AP_DAL_Baro::start_frame()
{
    const auto &baro = AP::baro();

    const log_RBRH old_RBRH = _RBRH;
    _RBRH.primary = baro.get_primary();
    _RBRH.num_instances = baro.num_instances();
    WRITE_REPLAY_BLOCK_IFCHANGED(RBRH, _RBRH, old_RBRH);

    for (uint8_t i=0; i<_RBRH.num_instances; i++) {
        log_RBRI &RBRI = _RBRI[i];
        log_RBRI old = RBRI;
        RBRI.last_update_ms = baro.get_last_update(i);
        RBRI.healthy = baro.healthy(i);
        RBRI.altitude = baro.get_altitude(i);
        WRITE_REPLAY_BLOCK_IFCHANGED(RBRI, _RBRI[i], old);
    }
}

void AP_DAL_Baro::update_calibration()
{
    AP::baro().update_calibration();
}
