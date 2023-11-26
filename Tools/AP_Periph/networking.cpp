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


void Networking_Periph::init(void)
{
    networking_lib.init();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    for (auto &p : passthru) {
        p.init();
    }
#endif
}

void Networking_Periph::update(void)
{
    networking_lib.update();

#if HAL_PERIPH_NETWORK_NUM_PASSTHRU > 0
    for (auto &p : passthru) {
        p.update();
    }
#endif
}

#endif  // HAL_PERIPH_ENABLE_NETWORKING

