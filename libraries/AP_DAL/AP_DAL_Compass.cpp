#include "AP_DAL_Compass.h"

#include <AP_Compass/AP_Compass.h>

#include <AP_Logger/AP_Logger.h>
#include "AP_DAL.h"

AP_DAL_Compass::AP_DAL_Compass()
{
    for (uint8_t i=0; i<ARRAY_SIZE(_RMGI); i++) {
        _RMGI[i].instance = i;
    }
}

void AP_DAL_Compass::start_frame()
{
    const auto &compass = AP::compass();

    const log_RMGH old = _RMGH;
    _RMGH.count = compass.get_count();
    _RMGH.auto_declination_enabled = compass.auto_declination_enabled();
    _RMGH.declination = compass.get_declination();
    _RMGH.num_enabled = compass.get_num_enabled();
    _RMGH.consistent = compass.consistent();
    _RMGH.first_usable = compass.get_first_usable();

    WRITE_REPLAY_BLOCK_IFCHANGED(RMGH, _RMGH, old);

    for (uint8_t i=0; i<_RMGH.count; i++) {
        log_RMGI &RMGI = _RMGI[i];
        const log_RMGI old_RMGI = RMGI;
        RMGI.use_for_yaw = compass.use_for_yaw(i);
        RMGI.healthy = compass.healthy(i);
        RMGI.offsets = compass.get_offsets(i);
        RMGI.have_scale_factor = compass.have_scale_factor(i);
        RMGI.last_update_usec = compass.last_update_usec(i);
        RMGI.field = compass.get_field(i);

        WRITE_REPLAY_BLOCK_IFCHANGED(RMGI, RMGI, old_RMGI);
    }
}
