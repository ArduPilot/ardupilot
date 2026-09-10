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

//
//  DDS GPS driver
//

#include "AP_GPS_config.h"

#if AP_GPS_DDS_ENABLED

#include "AP_GPS_DDS.h"

#include <AP_HAL/AP_HAL.h>

// Reading does nothing in this class; we simply return whether or not
// the latest reading has been consumed.  By calling this function we assume
// the caller is consuming the new data;
bool AP_GPS_DDS::read(void)
{
    if (_new_data) {
        _new_data = false;
        return true;
    }

    return false;
}

// handles a GPS sample pushed in over DDS and sets corresponding gps data
// appropriately;
void AP_GPS_DDS::handle_navsatfix(const NavSatFix &pkt)
{
    // This is a process boundary: the sample comes from a companion computer,
    // so a bad value here must not reach the EKF as a plausible-looking fix.
    // NaN fails every comparison below, so the range checks reject it too.
    if (!(pkt.latitude_deg >= -90.0 && pkt.latitude_deg <= 90.0) ||
        !(pkt.longitude_deg >= -180.0 && pkt.longitude_deg <= 180.0) ||
        !isfinite(pkt.altitude_amsl_m)) {
        return;
    }

    check_new_itow(pkt.gps_tow_ms, sizeof(pkt));

    state.time_week = pkt.gps_week;
    state.time_week_ms = pkt.gps_tow_ms;

    if (pkt.fix_type == AP_GPS_FixType::NO_GPS) {
        state.status = AP_GPS::NO_FIX;
    } else {
        state.status = (AP_GPS::GPS_Status)pkt.fix_type;
    }

    state.num_sats = pkt.num_sats;

    // (double) casts defeat -fsingle-precision-constant, which AP_GPS is built
    // with; round() and fabs() are unavailable on non-SITL boards, so the
    // rounding is done by hand here.
    const double lat_1e7 = pkt.latitude_deg * (double)1e7;
    const double lng_1e7 = pkt.longitude_deg * (double)1e7;

    const Location loc {
        int32_t(lat_1e7 + (lat_1e7 < 0 ? -0.5 : 0.5)),
        int32_t(lng_1e7 + (lng_1e7 < 0 ? -0.5 : 0.5)),
        int32_t(roundf(pkt.altitude_amsl_m * 100.0f)),   // cm
        Location::AltFrame::ABSOLUTE
    };
    state.location = loc;

    if (pkt.have_dop) {
        state.hdop = uint16_t(constrain_float(roundf(pkt.hdop * 100.0f), 0, UINT16_MAX));
        state.vdop = uint16_t(constrain_float(roundf(pkt.vdop * 100.0f), 0, UINT16_MAX));
    }

    if (pkt.have_velocity) {
        state.velocity = pkt.velocity_ned;
        if (!pkt.have_vertical_velocity) {
            state.velocity.z = 0;
        }
        state.have_vertical_velocity = pkt.have_vertical_velocity;
        velocity_to_speed_course(state);
    }

    if (pkt.have_accuracy) {
        state.horizontal_accuracy = pkt.horizontal_accuracy;
        state.vertical_accuracy = pkt.vertical_accuracy;
        state.speed_accuracy = pkt.speed_accuracy;
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;
        state.have_speed_accuracy = true;
    }

    state.last_gps_time_ms = AP_HAL::millis();

    _new_data = true;
}

#endif  // AP_GPS_DDS_ENABLED
