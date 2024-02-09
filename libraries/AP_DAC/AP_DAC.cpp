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

#include "AP_DAC.h"

#if AP_DAC_ENABLED

#include "AP_DAC.h"
#include "AP_DAC_Params.h"
#include "AP_DAC_TIx3204.h"

const AP_Param::GroupInfo AP_DAC::var_info[] = {

    // @Group: 1_
    // @Path: AP_DAC_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 1, AP_DAC, AP_DAC_Params),

    AP_GROUPEND
};

// Constructor
AP_DAC::AP_DAC()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  init - instantiate the DACs
*/
void AP_DAC::init()
{
    for (uint8_t i = 0; i < AP_DAC_MAX_INSTANCES; i++) {
        const AP_DAC_Params::Type type = params[i].type;
        switch (type) {
        case AP_DAC_Params::Type::TIx3204:
            backends[i] = new AP_DAC_TIx3204(params[i]);
            break;

        case AP_DAC_Params::Type::NONE:
            break;
        }
        if (backends[i] != nullptr) {
            backends[i]->init();
        }
    }
}

/*
  set output voltage on a channel
 */
bool AP_DAC::set_voltage(uint8_t instance, uint8_t channel, float voltage)
{
    if (instance >= AP_DAC_MAX_INSTANCES) {
        return false;
    }
    if (backends[instance] == nullptr) {
        return false;
    }
    return backends[instance]->set_voltage(channel, voltage);
}

#endif // AP_DAC_ENABLED
