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

#include "AP_IBus2_Masters.h"

#if AP_IBUS2_MASTER_ENABLED

const AP_Param::GroupInfo AP_IBus2_Masters::var_info[] = {
    // @Group: 1_
    // @Path: AP_IBus2_Master.cpp
    AP_SUBGROUPINFO(_instances[0], "1_", 1, AP_IBus2_Masters, AP_IBus2_Master),

#if AP_IBUS2_MAX_MASTER_INSTANCES >= 2
    // @Group: 2_
    // @Path: AP_IBus2_Master.cpp
    AP_SUBGROUPINFO(_instances[1], "2_", 2, AP_IBus2_Masters, AP_IBus2_Master),
#endif

#if AP_IBUS2_MAX_MASTER_INSTANCES >= 3
    // @Group: 3_
    // @Path: AP_IBus2_Master.cpp
    AP_SUBGROUPINFO(_instances[2], "3_", 3, AP_IBus2_Masters, AP_IBus2_Master),
#endif

#if AP_IBUS2_MAX_MASTER_INSTANCES >= 4
    // @Group: 4_
    // @Path: AP_IBus2_Master.cpp
    AP_SUBGROUPINFO(_instances[3], "4_", 4, AP_IBus2_Masters, AP_IBus2_Master),
#endif

    // @Param: CYC_US
    // @DisplayName: IBUS2 master cycle time
    // @Description: Time between IBUS2 master Frame 1/Frame 2 cycles
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("CYC_US", 5, AP_IBus2_Masters, _cycle_us, IBUS2_CYCLE_US),

    // @Param: RTO_US
    // @DisplayName: IBUS2 master response timeout
    // @Description: Time to wait for a Frame 3 response after sending Frame 2
    // @Units: us
    // @User: Advanced
    AP_GROUPINFO("RTO_US", 6, AP_IBus2_Masters, _response_timeout_us, IBUS2_RESPONSE_TIMEOUT_US),

    AP_GROUPEND
};

AP_IBus2_Masters::AP_IBus2_Masters()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_IBus2_Masters::init()
{
    static_assert(AP_IBUS2_MAX_MASTER_INSTANCES >= 1 &&
                  AP_IBUS2_MAX_MASTER_INSTANCES <= 4,
                  "AP_IBUS2_MAX_MASTER_INSTANCES must be 1-4");
    for (uint8_t i = 0; i < AP_IBUS2_MAX_MASTER_INSTANCES; i++) {
        _instances[i].init(i, _cycle_us, _response_timeout_us);
    }
}

#endif  // AP_IBUS2_MASTER_ENABLED
