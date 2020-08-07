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
  Simulator for the EcoEFI ECU
*/

#include <AP_Math/AP_Math.h>

#include "SIM_EcoEFI.h"
#include "SITL.h"
#include <AP_HAL/utility/sparse-endian.h>

#include <stdio.h>
#include <errno.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo EcoEFI::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: EcoEFI sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the EcoEFI simulator
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, EcoEFI, _enabled, 0),

    AP_GROUPEND
};

EcoEFI::EcoEFI() : SerialDevice::SerialDevice()
{
    AP_Param::setup_object_defaults(this, var_info);

    u.packet.headermagic1 = 0x80;
    u.packet.headermagic2 = 0x8F;
    u.packet.headermagic3 = 0xEA;
    u.packet.data_field_length = 0x1B;
    u.packet.service_id = 0x50;
}

void EcoEFI::update(const struct sitl_input &input)
{
    update_send();
}

#include <stdio.h>

void EcoEFI::EcoEFIUnion::update_checksum()
{
    packet.checksum = 0;
    for (uint8_t i=0; i<ARRAY_SIZE(u.parse_buffer)-1; i++) {
        packet.checksum += parse_buffer[i];
    }
}

void EcoEFI::update_send()
{
    auto sitl = AP::sitl();
    if (sitl == nullptr || sitl->efi_type == SITL::EFI_TYPE_NONE) {
        return;
    }

    // just send a chunk of data at 10Hz:
    const uint32_t now = AP_HAL::millis();
    if (now - last_sent_ms < 100) {
        return;
    }
    last_sent_ms = now;

    u.packet.timestamp += 1;
    u.packet.RPM = htobe16(sitl->state.rpm[0] * 0.25);
    u.packet.MAP = htobe16(AP::baro().get_pressure()/1000.0*0.9 * (1/0.0039));
    u.packet.TPS = htobe16(17 * (1/0.0015));
    u.packet.ECT = htobe16((15.0 +40) * (1/1.25));
    u.packet.IAT = htobe16((26.0 +40) * (1/1.25));
    u.packet.BARO = htobe16(AP::baro().get_pressure()/1000.0 * (1/0.0039));
    u.update_checksum();

    if (write_to_autopilot((char*)u.parse_buffer, ARRAY_SIZE(u.parse_buffer)) != ARRAY_SIZE(u.parse_buffer)) {
        AP_HAL::panic("Failed to write to autopilot: %s", strerror(errno));
    }
}
