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

/*
  Driver for ERDI TECH 905nm laser rangefinders with the 4-byte "5C"
  UART stream protocol:

    VB series (LRF10VB, LRF22VB, LRF50VB): distance in millimetres
    H series  (LRF50H, LRF100H, LRF200H, LRF300H, LRF600H): distance in centimetres

  Frame (4 bytes, 460800 8N1 by default):
    0x5C | DIST_L | DIST_H | CHECKSUM
  CHECKSUM = ~(DIST_L + DIST_H)
 */

#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_ERDI_ENABLED

#include "AP_RangeFinder_Backend_Serial.h"

class AP_RangeFinder_ERDI : public AP_RangeFinder_Backend_Serial
{
public:

    enum class Model : uint8_t {
        VB = 0,     // distance reported in millimetres
        H  = 1,     // distance reported in centimetres
    };

    static AP_RangeFinder_Backend_Serial *create_VB(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_ERDI(_state, _params, Model::VB);
    }

    static AP_RangeFinder_Backend_Serial *create_H(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_ERDI(_state, _params, Model::H);
    }

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:

    AP_RangeFinder_ERDI(RangeFinder::RangeFinder_State &_state,
                        AP_RangeFinder_Params &_params,
                        Model model) :
        AP_RangeFinder_Backend_Serial(_state, _params),
        _model(model)
    {}

    bool get_reading(float &reading_m) override;

    // factory default baud rate for both series
    uint32_t initial_baudrate(uint8_t serial_instance) const override { return 460800; }

    // VB series modules can be stopped by command; ask them to
    // (re)start ranging if we have not seen a valid frame for a while
    void send_start_command();

    // shift the line buffer to the next frame header after a bad frame
    void resync();

    const Model _model;
    uint8_t linebuf[4];
    uint8_t linebuf_len;
    uint32_t last_valid_frame_ms;
    uint32_t last_start_cmd_ms;
};

#endif  // AP_RANGEFINDER_ERDI_ENABLED
