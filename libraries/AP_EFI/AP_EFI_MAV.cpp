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
#include <AP_MATH/definitions.h>

AP_EFI_MAV::AP_EFI_MAV(AP_EFI &_frontend) :  AP_EFI_Backend(_frontend) {
    //Nothing to do here
}

//Called from frontend to update with the readings received by handler
void AP_EFI_MAV::update() {
    copy_to_frontend();
}

//Decode MavLink message
void AP_EFI_MAV::handle_EFI_message(const mavlink_message_t &msg) {
    mavlink_efi_status_t state;
    mavlink_msg_efi_status_decode(&msg, &state);

    internal_state.ecu_index = state.ecu_index;
    //internal_state.??? = state.rpm;
    //internal_state.??? = state.fuel_consumed;
    //internal_state.??? = state.fuel_flow;
    internal_state.engine_load_percent = state.engine_load;
    internal_state.throttle_position_percent = state.throttle_position;
    internal_state.spark_dwell_time_ms = state.spark_dwell_time;
    //internal_state.??? = state.barometric_pressure;
    internal_state.intake_manifold_pressure_kpa = state.intake_manifold_pressure;
    internal_state.intake_manifold_temperature = C_TO_KELVIN(state.intake_manifold_temperature);
    internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(state.cylinder_head_temperature);
    internal_state.cylinder_status.ignition_timing_deg = state.ignition_timing;
    internal_state.cylinder_status.injection_time_ms = state.injection_time;
    internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(state.exhaust_gas_temperature);
    internal_state.throttle_out = state.throttle_out;
    internal_state.pt_compensation = state.pt_compensation;
    //internal_state.??? = state.health;
    internal_state.ignition_voltage = state.ignition_voltage;
}
