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
  Simulator for the EcoEFI Serial connection

./Tools/autotest/sim_vehicle.py --gdb --debug -v ArduCopter -A --uartF=sim:ecoefi --speedup=1 --console

param set SERIAL5_PROTOCOL 32
param set EFI_TYPE 2

reboot

*/

#pragma once

#include <AP_Param/AP_Param.h>

#include "SITL_Input.h"

#include "SIM_SerialDevice.h"

#include <stdio.h>

namespace SITL {

class EcoEFI : public SerialDevice {
public:

    EcoEFI();

    // update state
    void update(const struct sitl_input &input);

    static const AP_Param::GroupInfo var_info[];

private:

    void update_send();

    AP_Int8  _enabled;  // enable richenpower sim

    // packet to send:
    struct PACKED EcoEFIPacket {
        uint8_t headermagic1;
        uint8_t headermagic2;
        uint8_t headermagic3;
        uint8_t data_field_length;
        uint8_t service_id;
        uint8_t timestamp;
        uint16_t RPM;
        uint16_t MAP;
        uint16_t TPS;
        uint16_t ECT;
        uint16_t IAT;
        uint16_t O2S;
        uint16_t SPARK;
        uint16_t FUELPW1;
        uint16_t FUELPW2;
        uint16_t UbAdc;
        uint8_t FuelLvl;
        uint16_t BARO;
        uint16_t Field_Consumption;
        uint8_t checksum;
    };
    assert_storage_size<EcoEFIPacket, 32> _assert_storage_size_EcoEFIPacket;

    union EcoEFIUnion {
        uint8_t parse_buffer[32];
        struct EcoEFIPacket packet;

        void update_checksum();
    };
    assert_storage_size<EcoEFIUnion, 32> _assert_storage_size_EcoEFIUnion;

    EcoEFIUnion u;

    uint32_t last_sent_ms;
};

}
