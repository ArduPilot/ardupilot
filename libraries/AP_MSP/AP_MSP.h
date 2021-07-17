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
   MSP version 1 and 2 protocol library, based on betaflight/iNav implementations
   code by Alex Apostoli
 */

#pragma once

#include <AP_OSD/AP_OSD.h>

#include "AP_MSP_Telem_Backend.h"

#if HAL_MSP_ENABLED

#define MSP_MAX_INSTANCES 3
#define MSP_OSD_START 2048
#define MSP_OSD_STEP_X 1
#define MSP_OSD_STEP_Y 32
#define MSP_OSD_POS(osd_setting) (MSP_OSD_START + osd_setting->xpos*MSP_OSD_STEP_X + osd_setting->ypos*MSP_OSD_STEP_Y)

class AP_MSP
{
    friend class AP_MSP_Telem_Generic;
    friend class AP_MSP_Telem_DJI;
    friend class AP_MSP_Telem_Backend;

public:
    AP_MSP();

    /* Do not allow copies */
    AP_MSP(const AP_MSP &other) = delete;
    AP_MSP &operator=(const AP_MSP&) = delete;

    // User settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // init - perform required initialisation
    void init();

    static AP_MSP *get_singleton(void)
    {
        return _singleton;
    }

private:

    enum msp_option_e : uint8_t {
        OPTION_TELEMETRY_MODE = 1U<<0,
        OPTION_TELEMETRY_DJI_WORKAROUNDS = 1U<<1
    };

    AP_MSP_Telem_Backend *_backends[MSP_MAX_INSTANCES];

    AP_Int8 _options;
    AP_Int8 _cellcount;

    // these are the osd items we support for MSP OSD
    AP_OSD_Setting* _osd_item_settings[MSP::OSD_ITEM_COUNT];
    MSP::osd_config_t _osd_config;

    struct {
        bool flashing_on;                                       // OSD item flashing support
        uint8_t last_flight_mode = 255;
        uint32_t last_flight_mode_change_ms;
        bool flight_mode_focus;                                 // do we need to steal focus from text messages
        bool osd_initialized;                                   // for one time osd initialization
        uint8_t backend_count;                                  // actual count of active bacends
    } _msp_status;

    bool init_backend(uint8_t backend_idx, AP_HAL::UARTDriver *uart, AP_SerialManager::SerialProtocol protocol);
    void init_osd();
    void loop(void);
    bool check_option(const msp_option_e option);

    static AP_MSP *_singleton;
};

namespace AP
{
AP_MSP *msp();
};

#endif //HAL_MSP_ENABLED
