#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_COMPANION_ENABLED

#include "AC_PrecLand_Backend.h"
#include <AP_Math/AP_Math.h>

/*
 * AC_PrecLand_Companion - implements precision landing using target vectors provided
 *                         by a companion computer (i.e. Odroid) communicating via MAVLink
 *                         The companion computer must provide "Line-Of-Sight" measurements
 *                         in the form of LANDING_TARGET mavlink messages.
 */

class AC_PrecLand_Companion : public AC_PrecLand_Backend
{
public:
    // Constructor
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

    // parses a mavlink message from the companion computer
    void handle_msg(const mavlink_landing_target_t &packet, uint32_t timestamp_ms) override;

private:
    bool                _wrong_frame_msg_sent;
};


#endif // AC_PRECLAND_COMPANION_ENABLED
