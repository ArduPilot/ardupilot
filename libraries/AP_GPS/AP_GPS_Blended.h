#pragma once

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_BLENDED_ENABLED

#include "AP_GPS.h"
#include "GPS_Backend.h"

class AP_GPS_Blended : public AP_GPS_Backend
{
public:

    AP_GPS_Blended(AP_GPS &_gps, AP_GPS::Params &_params, AP_GPS::GPS_State &_state, class AP_GPS::GPS_timing &_timing) :
        AP_GPS_Backend(_gps, _params, _state, nullptr),
        timing{_timing}
        { }

    // pre-arm check of GPS blending.  False if blending is unhealthy,
    // True if healthy or blending is not being used
    bool is_healthy() const override {
        return (_blend_health_counter < 50);
    }

    bool read() override { return true; }

    const char *name() const override { return "Blended"; }

    bool get_lag(float &lag_sec) const override;
    const Vector3f &get_antenna_offset() const override {
        return _blended_antenna_offset;
    }

    // calculate the blend weight.  Returns true if blend could be
    // calculated, false if not
    bool calc_weights(void);
    // calculate the blended state
    void calc_state(void);

    void zero_health_counter() {
        _blend_health_counter = 0;
    }

    AP_GPS::GPS_Status highest_supported_status(void) override {
        return _highest_supported_status;
    }

private:

    // GPS blending and switching
    Vector3f _blended_antenna_offset; // blended antenna offset
    float _blended_lag_sec; // blended receiver lag in seconds
    float _blend_weights[GPS_MAX_INSTANCES]; // blend weight for each GPS. The blend weights must sum to 1.0 across all instances.
    uint8_t _blend_health_counter;  // 0 = perfectly health, 100 = very unhealthy

    AP_GPS::GPS_timing &timing;
    bool _calc_weights(void);

    AP_GPS::GPS_Status _highest_supported_status;

    float new_timing_last_message_time_ms;

    // returns true if instance at offset i should be included in this blend
    bool should_include_instance_in_blend(uint8_t i) const;
};

#endif  // AP_GPS_BLENDED_ENABLED
