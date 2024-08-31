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
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_Serial_EcoEFI: public AP_EFI_Backend {

public:
    // Constructor with initialization
    AP_EFI_Serial_EcoEFI(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:

    AP_HAL::UARTDriver *uart;

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
    };
    assert_storage_size<EcoEFIUnion, 32> _assert_storage_size_EcoEFIUnion;

    EcoEFIUnion u;

    uint8_t body_length;

    // move the expected header bytes into &buffer[0], adjusting
    // body_length as appropriate.
    void move_header_in_buffer(uint8_t initial_offset);

    // the datasheet specifies *all* of these as constants.  5 magic
    // bytes seems a bit much :-)
    static constexpr uint8_t HEADER_MAGIC1 = 0x80;
    static constexpr uint8_t HEADER_MAGIC2 = 0x8F;
    static constexpr uint8_t HEADER_MAGIC3 = 0xEA;
    static constexpr uint8_t DATA_FIELD_LENGTH = 0x1B;
    static constexpr uint8_t SERVICE_ID = 0x50;
};
