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

// Datasheet: https://en.benewake.com/DataDownload/index_pid_20_lcid_104.html

#if AP_RANGEFINDER_BENEWAKE_TFA1500_ENABLED

#include "AP_RangeFinder_Benewake.h"

class AP_RangeFinder_Benewake_TFA1500 : public AP_RangeFinder_Benewake
{
public:
    static AP_RangeFinder_Backend_Serial *create(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params)
    {
        return NEW_NOTHROW AP_RangeFinder_Benewake_TFA1500(_state, _params);
    }

protected:
    float model_dist_max_cm() const override { return 0x3FFFFF; }

private:
    using AP_RangeFinder_Benewake::AP_RangeFinder_Benewake;
    bool get_reading(float &reading_m) override;
    void find_signature_in_buffer(uint8_t start);

    union {
        uint8_t bytes[5];
        struct PACKED {
            uint8_t header;
            uint8_t dist_low;
            uint8_t dist_mid;
            uint8_t dist_high;
            uint8_t checksum_of_bytes;
        } packet;
    } tf_frame;

    uint8_t tf_frame_len;
    uint32_t last_init_ms;
};

#endif // AP_RANGEFINDER_BENEWAKE_TFA1500
