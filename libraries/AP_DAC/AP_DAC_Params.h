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

#include <AP_Param/AP_Param.h>
#include "AP_DAC_config.h"

class AP_DAC_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_DAC_Params(void);

    CLASS_NO_COPY(AP_DAC_Params);

    // DAC types
    enum class Type : uint8_t {
        NONE      = 0,
#if AP_DAC_TIX3204_ENABLED
        TIx3204   = 1,
#endif
#if AP_DAC_MCP40D1X_ENABLED
        MCP40D1x   = 2,
#endif
    };

    AP_Enum<Type> type;             // 0=disabled, others see frontend enum TYPE
    AP_Int8 bus;                    // I2C bus number
    AP_Int8 bus_address;            // I2C address
    AP_Float voltage_reference;
    AP_Float voltage;
};
