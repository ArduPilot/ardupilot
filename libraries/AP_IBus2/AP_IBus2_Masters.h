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
  AP_IBus2_Masters: container for AP_IBus2_Master instances.

  Owns the per-instance parameter subgroups (IBUS2M_n_*) and the timing
  parameters shared by all instances.  Instance n uses the serial port
  configured with SerialProtocol_IBUS2_Master ordinal n.
*/

#pragma once

#include "AP_IBus2_config.h"

#if AP_IBUS2_MASTER_ENABLED

#include "AP_IBus2_Master.h"
#include <AP_Param/AP_Param.h>

class AP_IBus2_Masters
{
public:
    AP_IBus2_Masters();

    CLASS_NO_COPY(AP_IBus2_Masters);

    // init each enabled instance; each registers its own timer callback
    void init();

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_IBus2_Master _instances[AP_IBUS2_MAX_MASTER_INSTANCES];

    // Timing parameters (µs) shared by all master instances
    AP_Int32 _cycle_us;
    AP_Int32 _response_timeout_us;
};

#endif  // AP_IBUS2_MASTER_ENABLED
