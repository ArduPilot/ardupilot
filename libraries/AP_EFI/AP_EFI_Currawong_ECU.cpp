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
 * AP_EFI_Currawong_ECU.cpp
 *
 *      Author: Reilly Callaway
 */

#include <AP_PiccoloCAN/piccolo_protocol/ECUPackets.h>
#include <AP_Math/definitions.h>

#if HAL_EFI_CURRAWONG_ECU_ENABLED

extern const AP_HAL::HAL& hal;

AP_EFI_Currawong_ECU* AP_EFI_Currawong_ECU::singleton;

AP_EFI_Currawong_ECU::AP_EFI_Currawong_ECU(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    singleton = this;

    internal_state.oil_pressure_status = Oil_Pressure_Status::OIL_PRESSURE_STATUS_NOT_SUPPORTED;
    internal_state.debris_status = Debris_Status::NOT_SUPPORTED;
    internal_state.misfire_status = Misfire_Status::NOT_SUPPORTED;
}

void AP_EFI_Currawong_ECU::update()
{
    // copy the data to the front end
    copy_to_frontend();
}

bool AP_EFI_Currawong_ECU::handle_message(AP_HAL::CANFrame &frame)
{
    bool valid  = true;

    // There are differences between Ardupilot EFI_State and types/scaling of Piccolo packets.
    // So we first decode to Piccolo structs, and then store the data we need in EFI_State internal_state with any scaling required.

    // Structs to decode Piccolo messages into
    ECU_TelemetryFast_t telemetryFast;
    ECU_TelemetrySlow0_t telemetrySlow0;
    ECU_TelemetrySlow1_t telemetrySlow1;
    ECU_TelemetrySlow2_t telemetrySlow2;
    ECU_Errors_t errors;

    // Throw the message at the decoding functions


    if (valid)
    {
        internal_state.last_updated_ms = AP_HAL::millis();
    }

    return valid;
}

#endif // HAL_EFI_CURRAWONG_ECU_ENABLED