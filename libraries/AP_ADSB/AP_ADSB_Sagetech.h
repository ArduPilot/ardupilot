#pragma once

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

#include "AP_ADSB_Backend.h"

//class AP_ADSB_Sagetech : public AP_ADSB_Backend {
//public:
//    // constructor
//    AP_ADSB_Sagetech(AP_ADSB &frontend);
//
//private:
//
//    struct PACKED header {
//        const uint8_t   start = 0xA5;
//        const uint8_t   assemblyAddress = 0x01;
//        uint8_t         message_type;
//        uint8_t         message_id;
//        uint8_t         payload_length;
//        //uint8_t*        payload;
//        uint8_t         checksumFletcher;
//        uint8_t         checksum;
//        const uint8_t   end = 0x5A;
//    };
//
//    struct payload_installation {
//        const uint8_t type = 0x01;
//        const uint8_t len = 28;
//        struct PACKED {
//            uint8_t header1;
//        } val;
//    };

//};

