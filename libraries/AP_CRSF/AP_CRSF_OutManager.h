/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andy Piper <github@andypiper.com>
 */

/*
 * AP_CRSF_Out.h - High-level driver for CRSF RC Output
 */
#pragma once

#include "AP_CRSF_config.h"

#if AP_CRSF_OUT_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

class AP_CRSF_Out;
class AP_CRSF_Protocol;

class AP_CRSF_OutManager {
    friend class AP_CRSF_Out;

public:
    // constructor for configuration
    AP_CRSF_OutManager();
    
    ~AP_CRSF_OutManager();

    // constructor for serial interaction
    AP_CRSF_Out* create_instance(AP_HAL::UARTDriver& uart);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_CRSF_OutManager);

    void init();

    // callback from the RC thread
    void update();

    static const struct AP_Param::GroupInfo var_info[];

private:
    uint8_t _num_instances;
    AP_CRSF_Protocol* _instances[SERIALMANAGER_NUM_PORTS];

    AP_Int16 _rate_hz;
};

#endif // AP_CRSF_OUT_ENABLED