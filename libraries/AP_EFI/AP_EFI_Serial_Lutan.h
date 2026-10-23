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
  support for Lutan serial EFI
 */
#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_LUTAN_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

class AP_EFI_Serial_Lutan: public AP_EFI_Backend {
    
public:
    // Constructor with initialization
    AP_EFI_Serial_Lutan(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;
    void send_request(void);

    union {
        struct PACKED {
            // fields from channels in LUTAN-MS2.ini
            uint16_t length;
            uint8_t flags;
            uint16_t seconds;
            uint16_t pulseWidth1; // ms, scale 0.000666
            uint16_t pulseWidth2; // ms, scale 0.000666
            uint16_t rpm;
            int16_t advance;  // deg, scale 0.1
            uint8_t squirt_flags;
            uint8_t engine_flags;
            uint8_t afrtgt1;
            uint8_t afrtgt2;
            uint8_t wbo2_en1;
            uint8_t wbo2_en2;
            int16_t barometer; // kPa, scale 0.1
            int16_t map; // kPa, scale 0.1
            int16_t mat; // degF, scale 0.1
            int16_t coolant; // degF, scale 0.1
            int16_t tps; // %, scale 0.1
            int16_t batteryVoltage; // V, scale 0.1
            int16_t afr1; // scale 0.1
            int16_t afr2; // scale 0.1
            uint16_t knock; // %, scale 0.1
            int16_t egoCorrection1; // %, scale 0.1
            int16_t egoCorrection2; // %, scale 0.1
            int16_t airCorrection; // %, scale 0.1
            int16_t warmupEnrich; // %, scale 0.1
            int16_t accelEnrich; // ms, scale 0.1
            int16_t tpsfuelcut; // %
            int16_t baroCorrection; // %, scale 0.1
            int16_t gammaEnrich; // %
            int16_t veCurr1; // %, scale 0.1
            int16_t veCurr2; // %, scale 0.1
            int16_t iacstep;
            int16_t idleDC; // scale 0.392
            int16_t coldAdvDeg; // deg, scale 0.1
            int16_t TPSdot; // %/s, scale 0.1
            int16_t MAPdot; // kPa/s
            int16_t dwell; // ms, scale 0.0666
        } data;
        uint8_t pkt[400];
    };
    uint16_t pkt_nbytes;
    uint32_t last_request_ms;
    uint32_t last_recv_ms;
};

#endif  // AP_EFI_SERIAL_LUTAN_ENABLED
