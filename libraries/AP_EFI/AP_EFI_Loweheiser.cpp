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

#include "AP_EFI_Loweheiser.h"

#if AP_EFI_LOWEHEISER_ENABLED

#include <AP_Generator/AP_Generator.h>
#include <AP_Generator/AP_Generator_Loweheiser.h>

// The Loweheiser EFI backend can be unhealthy not simply because
// we've not gotten an update, but also because it is rejecting our
// (mavlink) commands:
bool AP_EFI_Loweheiser::healthy() const
{
    AP_Generator *gen = AP::generator();
    if (gen == nullptr) {
        return false;
    }

    AP_Generator_Loweheiser *backend = gen->get_loweheiser();
    if (backend == nullptr) {
        return false;
    }

    if (!backend->good_ack) {
        return false;
    }

    return AP_EFI_Backend::healthy();
}

void AP_EFI_Loweheiser::update()
{
    AP_Generator *gen = AP::generator();
    if (gen == nullptr) {
        return;
    }

    AP_Generator_Loweheiser *backend = gen->get_loweheiser();
    if (backend == nullptr) {
        return;
    }

    if (backend->last_packet_received_ms == internal_state.last_updated_ms) {
        return;
    }

    // AP_Generator_Loweheiser remembers the last packet we got from
    // the generator as a convenient way of storing data we may wish
    // to relay:
    const mavlink_loweheiser_gov_efi_t &pkt = backend->packet;

    // this list is ordered by the field order in the
    // LOWEHEISER_GOV_EFI message.  These fields become NaN when the
    // EFI is powered off.  This is the case when the generator
    // control switch is in the "don't run" position.
    if (isnan(pkt.throttle)) {
        // this is an integer field in internal_state
        internal_state.throttle_out = 0;
    } else {
        internal_state.throttle_out = pkt.throttle;
    }
    // pkt.efi_batt is used in battery monitoring
    if (isnan(pkt.efi_rpm)) {
        // this is an integer field in internal_state
        internal_state.engine_speed_rpm = 0;
    } else {
        internal_state.engine_speed_rpm = pkt.efi_rpm;
    }
    internal_state.cylinder_status.injection_time_ms = pkt.efi_pw;
    // we convert to cubic-cm-per-minute here from litres/second here:
    internal_state.fuel_consumption_rate_cm3pm = pkt.efi_fuel_flow*(1000/60.0);
    internal_state.estimated_consumed_fuel_volume_cm3 = backend->fuel_consumed() * 1000;;
    internal_state.intake_manifold_pressure_kpa = pkt.efi_baro;
    internal_state.intake_manifold_temperature = C_TO_KELVIN(pkt.efi_mat);
    internal_state.cylinder_status.cylinder_head_temperature = C_TO_KELVIN(pkt.efi_clt);
    internal_state.cylinder_status.exhaust_gas_temperature = C_TO_KELVIN(pkt.efi_exhaust_gas_temperature);
    if (isnan(pkt.efi_tps)) {
        // this is an integer field in internal_state
        internal_state.throttle_position_percent = 0;
    } else {
        internal_state.throttle_position_percent = pkt.efi_tps;
    }
    internal_state.ignition_voltage = pkt.efi_batt;

    internal_state.last_updated_ms = backend->last_packet_received_ms;

    // copy the data to the front end
    copy_to_frontend();
}

#endif // AP_EFI_LOWEHEISER_ENABLED
