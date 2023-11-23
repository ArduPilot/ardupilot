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

#ifdef HAL_PERIPH_ENABLE_NETWORKING

#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Networking_Periph::var_info[] {
    // @Group:
    // @Path: ../../libraries/AP_Networking/AP_Networking.cpp
    AP_SUBGROUPINFO(networking_lib, "", 1, Networking_Periph, AP_Networking),

    /*
      the NET_Pn_ parameters need to be here as otherwise we
      are too deep in the parameter tree
     */

#if AP_NETWORKING_NUM_PORTS > 0
    // @Group: P1_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[0], "P1_", 2, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 1
    // @Group: P2_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[1], "P2_", 3, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 2
    // @Group: P3_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[2], "P3_", 4, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 3
    // @Group: P4_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[3], "P4_", 5, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 4
    // @Group: P5_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[4], "P5_", 6, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 5
    // @Group: P6_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[5], "P6_", 7, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 6
    // @Group: P7_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[6], "P7_", 8, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 7
    // @Group: P8_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[7], "P8_", 9, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 8
    // @Group: P9_
    // @Path: ../../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking_lib.ports[8], "P9_", 10, Networking_Periph, AP_Networking::Port),
#endif



#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    // @Group: PASS1_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[0], "PASS1_", 11, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 1
    // @Group: PASS2_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[1], "PASS2_", 12, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 2
    // @Group: PASS3_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[2], "PASS3_", 13, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 3
    // @Group: PASS4_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[3], "PASS4_", 14, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 4
    // @Group: PASS5_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[4], "PASS5_", 15, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 5
    // @Group: PASS6_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[5], "PASS6_", 16, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 6
    // @Group: PASS7_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[6], "PASS7_", 17, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 7
    // @Group: PASS8_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[7], "PASS8_", 18, Networking_Periph, Passthru),
#endif

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 8
    // @Group: PASS9_
    // @Path: networking_passthru.cpp
    AP_SUBGROUPINFO(passthru[8], "PASS9_", 19, Networking_Periph, Passthru),
#endif

    AP_GROUPEND
};


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

void Networking_Periph::init(void)
{
    networking_lib.init();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    auto &serial_manager = AP::serialmanager();

    for (auto &p : passthru) {
        if (p.enabled != 0 && p.port1 == nullptr && p.port2 == nullptr &&
            p.ep1 != -1 && p.ep2 != -1 && p.ep1 != p.ep2) {

            p.port1 = serial_manager.get_serial_by_id(p.ep1);
            p.port2 = serial_manager.get_serial_by_id(p.ep2);

            if (p.port1 != nullptr && p.port2 != nullptr) {
                p.port1->set_options(p.options1);
                p.port1->begin(p.baud1);

                p.port2->set_options(p.options2);
                p.port2->begin(p.baud2);
            }
        }
    }
#endif // HAL_PERIPH_NETWORK_NUM_PASSTHRU
}

void Networking_Periph::update(void)
{
    networking_lib.update();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    for (auto &p : passthru) {
        if (p.enabled == 0 || p.port1 == nullptr || p.port2 == nullptr) {
            continue;
        }
        uint8_t buf[1024];

        // read from port1, and write to port2
        auto avail = p.port1->available();
        if (avail > 0) {
            auto space = p.port2->txspace();
            const uint32_t n = MIN(space, sizeof(buf));
            const auto nbytes = p.port1->read(buf, n);
            if (nbytes > 0) {
                p.port2->write(buf, nbytes);
            }
        }

        // read from port2, and write to port1
        avail = p.port2->available();
        if (avail > 0) {
            auto space = p.port1->txspace();
            const uint32_t n = MIN(space, sizeof(buf));
            const auto nbytes = p.port2->read(buf, n);
            if (nbytes > 0) {
                p.port1->write(buf, nbytes);
            }
        }
    }
#endif // HAL_PERIPH_NETWORK_NUM_PASSTHRU
}

#endif  // HAL_PERIPH_ENABLE_NETWORKING

