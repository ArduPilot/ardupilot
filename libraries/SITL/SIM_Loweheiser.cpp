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
  Simulator for the Loweheiser generators
*/

#include <AP_Math/AP_Math.h>

#include "SIM_Loweheiser.h"
#include "SITL.h"

#include <stdio.h>
#include <errno.h>

using namespace SITL;

Loweheiser::Loweheiser() : SerialDevice::SerialDevice()
{
}

void Loweheiser::update()
{
    // if (!_enabled.get()) {
    //     return;
    // }
    maybe_send_heartbeat();
    update_send();
}

void Loweheiser::maybe_send_heartbeat()
{
    const uint32_t now = AP_HAL::millis();

    if (now - last_heartbeat_ms < 100) {
        // we only provide a heartbeat every so often
        return;
    }
    last_heartbeat_ms = now;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id,
                               component_id,
                               &msg,
                               MAV_TYPE_GCS,
                               MAV_AUTOPILOT_INVALID,
                               0,
                               0,
                               0);

    uint8_t buf[300];
    uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);

    if (::write(fd_my_end, (void*)&buf, buf_len) != buf_len) {
        ::fprintf(stderr, "write failure\n");
    }
}

void Loweheiser::update_send()
{
    // just send mavlink message at 10Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 100) {
        return;
    }
    last_sent_ms = now;

    mavlink_message_t msg;

    auto sitl = AP::sitl();
    if (!sitl || sitl->efi_type == SIM::EFI_TYPE_NONE) {
        return;
    }

    const float efi_rpm = 17;

    mavlink_msg_generator_status_pack_chan(
        system_id,
        component_id,
        mavlink_ch,
        &msg,
        0,  // status
        1234,  // generator speed
        0.1, // battery current
        0.2, // load current
        0.3, // power generated
        17.0, // bus voltage
        -134, // rectifier_temperature
        0.5, // batt current setpoint
        134, // generator_temperature
        10,  // runtime
        1  // time until maintenance required
        );

    {
        uint8_t buf[300];
        uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);

        if (write_to_autopilot((char*)buf, buf_len) != buf_len) {
            AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
        }
    }


    mavlink_msg_efi_status_pack_chan(
        system_id,
        component_id,
        mavlink_ch,
        &msg,
        1, // health
        0, // ecu_index,
        efi_rpm, // engine_speed_rpm,
        300, // estimated_consumed_fuel_volume_cm3,
        121, // fuel_consumption_rate_cm3pm,
        44, // engine_load_percent,
        12, // throttle_position_percent,
        134, // spark_dwell_time_ms,
        11, // atmospheric_pressure_kpa,
        12, // intake_manifold_pressure_kpa,
        13, // intake_manifold_temperature,
        14, // cylinder_head_temperature,
        15, // ignition_timing_deg,
        16, // cylinder_status.injection_time_ms,
        17, // exhaust gas temperature
        18, // throttle out
        19 //  pt compensation
        );
    {
        uint8_t buf[300];
        uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);

        if (write_to_autopilot((char*)buf, buf_len) != buf_len) {
            AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
        }
    }

}
