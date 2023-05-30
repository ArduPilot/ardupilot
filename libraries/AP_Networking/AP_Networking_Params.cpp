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

#include "AP_Networking_Params.h"
#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED

const AP_Param::GroupInfo AP_Networking_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Network Feature
    // @Description: Network Feature
    // @Values: 0:Disabled, 1:Speed Test, 2:LatencyTest, 3:Ping Out
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_Networking_Params, type, (int8_t)AP_Networking_Params::Type::None, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};

AP_Networking_Params::AP_Networking_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_NETWORKING_ENABLED

