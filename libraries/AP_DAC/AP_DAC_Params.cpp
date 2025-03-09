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

#include "AP_DAC_Params.h"
#include "AP_DAC.h"

#if AP_DAC_ENABLED

#ifndef AP_DAC_DEFAULT_BUS
#define AP_DAC_DEFAULT_BUS 0
#endif

#ifndef AP_DAC_DEFAULT_ADDR
#define AP_DAC_DEFAULT_ADDR 0
#endif

#ifndef AP_DAC_DEFAULT_VREF
#define AP_DAC_DEFAULT_VREF 3.3
#endif

#ifndef AP_DAC_DEFAULT_VOLT
#define AP_DAC_DEFAULT_VOLT 3.3
#endif

#ifndef AP_DAC_DEFAULT_TYPE
#define AP_DAC_DEFAULT_TYPE int8_t(Type::NONE)
#endif

const AP_Param::GroupInfo AP_DAC_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: DAC Type
    // @Description: DAC Type
    // @Values: 0:Disabled, 1:TIx3204, 2:MCP401x
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_DAC_Params, type, AP_DAC_DEFAULT_TYPE, AP_PARAM_FLAG_ENABLE),

    // @Param: BUS
    // @DisplayName: I2C bus
    // @Description: I2C bus number
    // @Range: 0 3
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("BUS", 2, AP_DAC_Params, bus, AP_DAC_DEFAULT_BUS),

    // @Param: ADDR
    // @DisplayName: I2C address
    // @Description: I2C address
    // @Range: 0 127
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("ADDR", 3, AP_DAC_Params, bus_address, AP_DAC_DEFAULT_ADDR),

    // @Param: VREF
    // @DisplayName: Voltage reference
    // @Description: Voltage reference
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("VREF", 4, AP_DAC_Params, voltage_reference, AP_DAC_DEFAULT_VREF),
    
    // @Param: VOLTS
    // @DisplayName: Voltage
    // @Description: Voltage
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("VOLTS", 5, AP_DAC_Params, voltage, AP_DAC_DEFAULT_VOLT),

    AP_GROUPEND
};

AP_DAC_Params::AP_DAC_Params(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_DAC_ENABLED
