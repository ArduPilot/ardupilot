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

#include "AP_Frsky_config.h"

#if AP_FRSKY_TELEM_ENABLED

#include "AP_Frsky_Backend.h"
#include "AP_Frsky_SPort.h"

class AP_Frsky_Parameters;

class AP_Frsky_Telem {

public:
    AP_Frsky_Telem();

    ~AP_Frsky_Telem();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Frsky_Telem);

    // init - perform required initialisation
    bool init(bool use_external_data=false);

    static AP_Frsky_Telem *get_singleton(void) {
        return singleton;
    }

    // get next telemetry data for external consumers of SPort data
    static bool get_telem_data(AP_Frsky_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data
    static bool set_telem_data(const uint8_t frame,const uint16_t appid, const uint32_t data);
#endif

    void queue_message(MAV_SEVERITY severity, const char *text) {
        if (_backend == nullptr) {
            return;
        }
        return _backend->queue_text_message(severity, text);
    }

private:

    AP_Frsky_Backend *_backend;
    AP_Frsky_Parameters* _frsky_parameters;

    // get next telemetry data for external consumers of SPort data (internal function)
    bool _get_telem_data(AP_Frsky_Backend::sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // set telemetry data from external producer of SPort data (internal function)
    bool _set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data);
#endif
    static void try_create_singleton_for_external_data(void);
    static AP_Frsky_Telem *singleton;

};

namespace AP {
    AP_Frsky_Telem *frsky_telem();
};

#endif  // AP_FRSKY_TELEM_ENABLED
