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

#include "AP_Periph.h"

#if defined(HAL_PERIPH_ENABLE_NETWORKING) && HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0

#include <AP_SerialManager/AP_SerialManager.h>

const AP_Param::GroupInfo Networking_Periph::Passthru::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable Passthrough
    // @Description: Enable Passthrough of any UART, Network, or CAN ports to any UART, Network, or CAN ports.
    // @Values: 0:Disabled, 1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1,  Networking_Periph::Passthru, enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: EP1
    // @DisplayName: Endpoint 1
    // @Description: Passthrough Endpoint 1. This can be a serial port UART, a Network port, or a CAN port. The selected port will route to Endport 2.
    // @Values: -1:Disabled, 0:Serial0(usually USB), 1:Serial1, 2:Serial2, 3:Serial3, 4:Serial4, 5:Serial5, 6:Serial6, 7:Serial7, 8:Serial8, 9:Serial9, 21:Network Port1, 22:Network Port2, 23:Network Port3, 24:Network Port4, 25:Network Port5, 26:Network Port6, 27:Network Port7, 28:Network Port8, 29:Network Port9, 41:CAN1 Port1, 42:CAN1 Port2, 43:CAN1 Port3, 44:CAN1 Port4, 45:CAN1 Port5, 46:CAN1 Port6, 47:CAN1 Port7, 48:CAN1 Port8, 49:CAN1 Port9, 51:CAN2 Port1, 52:CAN2 Port2, 53:CAN2 Port3, 54:CAN2 Port4, 55:CAN2 Port5, 56:CAN2 Port6, 57:CAN2 Port7, 58:CAN2 Port8, 59:CAN2 Port9
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("EP1", 2,  Networking_Periph::Passthru, ep1, -1),

    // @Param: EP2
    // @DisplayName: Endpoint 2
    // @Description: Passthrough Endpoint 2. This can be a serial port UART, a Network port, or a CAN port. The selected port will route to Endport 1.
    // @CopyFieldsFrom: NET_PASS1_EP1
    AP_GROUPINFO("EP2", 3,  Networking_Periph::Passthru, ep2, -1),

    // @Param: BAUD1
    // @DisplayName: Endpoint 1 Baud Rate
    // @Description: The baud rate used for Endpoint 1. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_BAUD
    AP_GROUPINFO("BAUD1", 4,  Networking_Periph::Passthru, baud1, 115200),

    // @Param: BAUD2
    // @DisplayName: Endpoint 2 Baud Rate
    // @Description: The baud rate used for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_BAUD
    AP_GROUPINFO("BAUD2", 5,  Networking_Periph::Passthru, baud2, 115200),

    // @Param: OPT1
    // @DisplayName: Serial Port Options EP1
    // @Description: Control over UART options for Endpoint 1. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_OPTIONS
    AP_GROUPINFO("OPT1", 6,  Networking_Periph::Passthru, options1, 0),

    // @Param: OPT2
    // @DisplayName: Serial Port Options EP2
    // @Description: Control over UART options for Endpoint 2. Only applies to serial ports.
    // @CopyFieldsFrom: SERIAL1_OPTIONS
    AP_GROUPINFO("OPT2", 7,  Networking_Periph::Passthru, options2, 0),

    AP_GROUPEND
};

void Networking_Periph::Passthru::init()
{
    if (enabled == 0) {
        // Feature is disabled
        return;
    }

    if (port1 != nullptr || port2 != nullptr) {
        // The ports have already been initialized, nothing to do.
        return;
    }

    if (ep1 <= -1 || ep2 <= -1 || ep1 == ep2) {
        // end points are not set or are the same. Can't route to self
        return;
    }

    port1 = AP::serialmanager().get_serial_by_id(ep1);
    port2 = AP::serialmanager().get_serial_by_id(ep2);

    if (port1 != nullptr && port2 != nullptr) {
        port1->set_options(options1);
        port1->begin(baud1);

        port2->set_options(options2);
        port2->begin(baud2);
    }
}

void Networking_Periph::Passthru::update()
{
    if (enabled == 0 || port1 == nullptr || port2 == nullptr) {
        return;
    }

    // Fastest possible connection is 3Mbps serial port, which is roughly 300KB/s payload and we service this at <= 1kHz
    // Raising this any higher just causes excess stack usage which never gets used.
    uint8_t buf[300];

    // read from port1, and write to port2
    auto avail = port1->available();
    if (avail > 0) {
        auto space = port2->txspace();
        const uint32_t n = MIN(space, sizeof(buf));
        const auto nbytes = port1->read(buf, n);
        if (nbytes > 0) {
            port2->write(buf, nbytes);
        }
    }

    // read from port2, and write to port1
    avail = port2->available();
    if (avail > 0) {
        auto space = port1->txspace();
        const uint32_t n = MIN(space, sizeof(buf));
        const auto nbytes = port2->read(buf, n);
        if (nbytes > 0) {
            port1->write(buf, nbytes);
        }
    }
}

#endif  // defined(HAL_PERIPH_ENABLE_NETWORKING) && HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0

