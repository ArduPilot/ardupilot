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

#include <AP_Common/AP_FWVersion.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#include "AP_MSP.h"
#include "AP_MSP_Telem_DisplayPort.h"

#if HAL_WITH_MSP_DISPLAYPORT

extern const AP_HAL::HAL& hal;

using namespace MSP;

MSPCommandResult AP_MSP_Telem_DisplayPort::msp_process_out_fc_variant(sbuf_t *dst)
{
    const AP_MSP *msp = AP::msp();
    if (msp == nullptr) {
        return MSP_RESULT_ERROR;
    }
    // do we use backend specific symbols table?
    if (msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_BTFL_SYMBOLS) && !msp->is_option_enabled(AP_MSP::Option::DISPLAYPORT_INAV_SYMBOLS)) {
        sbuf_write_data(dst, BETAFLIGHT_IDENTIFIER, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    } else {
        sbuf_write_data(dst, ARDUPILOT_IDENTIFIER, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
    }

    return MSP_RESULT_ACK;
}

bool AP_MSP_Telem_DisplayPort::init_uart()
{
    if (_msp_port.uart != nullptr)  {
        // re-init port here for use in this thread
        // displayport needs a bigger tx buffer
        // rx buffer can be small for we mostly write
        _msp_port.uart->begin(0, 256, 768);
        return true;
    }
    return false;
}

#endif //HAL_WITH_MSP_DISPLAYPORT
