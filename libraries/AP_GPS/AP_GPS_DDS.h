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
//  DDS GPS driver which accepts gps position data pushed in over DDS by a
//  companion computer.
//
//  Like AP_GPS_MAV and AP_GPS_ExternalAHRS this backend is push-driven: data
//  arrives asynchronously via handle_navsatfix() and read() only reports
//  whether an unconsumed sample is pending.
//
#pragma once

#include "AP_GPS_config.h"

#if AP_GPS_DDS_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "AP_GPS.h"
#include "AP_GPS_FixType.h"
#include "GPS_Backend.h"

class AP_GPS_DDS : public AP_GPS_Backend {
public:

    using AP_GPS_Backend::AP_GPS_Backend;

    /*
      One GPS sample pushed in from a companion computer.

      This is deliberately a plain struct with no ROS or AP_DDS types in it, so
      that the backend builds without AP_DDS. It is a superset of
      sensor_msgs/msg/NavSatFix: a NavSatFix on its own carries neither
      velocity, DOP nor satellite count, none of which AP_GPS can synthesise,
      so the DDS subscriber that eventually calls this is responsible for
      combining NavSatFix with its companion velocity topic and filling this in.

      Frames and units are ArduPilot-native, not ROS-native. In particular
      velocity is NED, whereas ROS publishes ENU; the conversion belongs in the
      subscriber, not here.
     */
    struct NavSatFix {
        uint16_t gps_week;              // GPS week number, 0 if unknown
        uint32_t gps_tow_ms;            // GPS time of week (ms)
        AP_GPS_FixType fix_type;
        uint8_t num_sats;               // satellites visible

        double latitude_deg;            // WGS-84 degrees
        double longitude_deg;           // WGS-84 degrees
        float altitude_amsl_m;          // metres above mean sea level

        bool have_velocity;
        Vector3f velocity_ned;          // m/s, NED
        bool have_vertical_velocity;

        bool have_accuracy;
        float horizontal_accuracy;      // m, RMS
        float vertical_accuracy;        // m, RMS
        float speed_accuracy;           // m/s, RMS

        bool have_dop;
        float hdop;                     // dimensionless, e.g. 1.55
        float vdop;                     // dimensionless
    };

    bool read() override;

    // Populate state from a sample pushed in over DDS. Nothing calls this yet;
    // the DDS subscriber that will is Phase 2.
    void handle_navsatfix(const NavSatFix &pkt);

    const char *name() const override { return "DDS"; }

private:
    // initialised here because AP_GPS_Backend's inherited constructor does not
    // touch it and read() is reached before the first push
    bool _new_data { false };
};

#endif  // AP_GPS_DDS_ENABLED
