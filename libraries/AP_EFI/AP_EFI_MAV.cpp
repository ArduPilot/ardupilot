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

#include "AP_EFI_MAV.h"

#include <GCS_MAVLink/GCS.h>

AP_EFI_MAV *AP_EFI_MAV::singleton;

AP_EFI_MAV::AP_EFI_MAV()
{
    singleton = this;
}

void AP_EFI_MAV::handle_EFI_message(const mavlink_message_t &msg)
{
	mavlink_msg_efi_status_decode(&msg, &efi_data);
}

float AP_EFI_MAV::get_EFI_state(uint8_t index)
{
	switch(index)
	{
	case 0:
		return efi_data.ecu_index;
		break;
	case 1:
		return efi_data.rpm;
		break;
	case 2:
		return efi_data.fuel_consumed;
		break;
	case 3:
		return efi_data.fuel_flow;
		break;
	case 4:
		return efi_data.engine_load;
		break;
	case 5:
		return efi_data.throttle_position;
		break;
	case 6:
		return efi_data.spark_dwell_time;
		break;
	case 7:
		return efi_data.barometric_pressure;
		break;
	case 8:
		return efi_data.intake_manifold_pressure;
		break;
	case 9:
		return efi_data.intake_manifold_temperature;
		break;
	case 10:
		return efi_data.cylinder_head_temperature;
		break;
	case 11:
		return efi_data.ignition_timing;
		break;
	case 12:
		return efi_data.injection_time;
		break;
	case 13:
		return efi_data.exhaust_gas_temperature;
		break;
	case 14:
		return efi_data.throttle_out;
		break;
	case 15:
		return efi_data.pt_compensation;
		break;
	case 16:
		return efi_data.ignition_voltage;
		break;
	}
	return 0.0;
}

namespace AP {
AP_EFI_MAV &EFI_MAV()
{
    return *AP_EFI_MAV::get_singleton();
}
}
