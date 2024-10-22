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
  support for FlyHenry serial EFI
 */
#pragma once

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_FH_ENABLED

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

#include "../AP_ICEngine/AP_ICEngine.h"

#define degC_to_Kelvin(degC) 273.15+(float)(degC)
#define fdiv(x,y) ((float)x/(float)y)
#define tenth(x) fdiv(x,10)
#define hundth(x) fdiv(x,100)
#define thousnd(x) fdiv(x,1000)

class AP_EFI_Serial_FH: public AP_EFI_Backend {
    
    friend AP_ICEngine;
public:
    // Constructor with initialization
    AP_EFI_Serial_FH(AP_EFI &_frontend);

    // Update the state structure
    void update() override;

private:
    AP_HAL::UARTDriver *port;
    
    void send_request(void);
    
    bool process(uint32_t const ms);
    
    uint8_t sum8(uint8_t *buf, size_t len) {
        uint8_t sum = 0;
        for (size_t i = 0; i < len; i++) sum += buf[i];
        return sum;
    }

    float convertFuelConsumption(float fuelConsumption_lph) {
      const int // Conversion factors
        litersToCubicCentimeters = 1000,// 1 liter = 1000 cm³
        hoursToMinutes = 60;            // 1 hour = 60 minutes
      
      float // Convert l/h to cm³/m
        fuelConsumption_cmh = fuelConsumption_lph * litersToCubicCentimeters,// l/h to cm³/h
        fuelConsumption_cmm = fuelConsumption_cmh / hoursToMinutes;           // cm³/h to cm³/m
      
      return fuelConsumption_cmm;
    }

  union {
    struct PACKED {
      uint8_t hb1;  //0 0xA5 - Head byte 1
      uint8_t hb2;  //1 0x5A - Head byte 2
      uint8_t pid;  //2 0x01 - Packet ID
      uint8_t res;  //3 0x00 - Reserved

      int16_t temp[6];  //4-15 Temperature 1-6 [Deg°C]
      int16_t tempOut;  //16 Outside temperature [Deg°C]

      uint16_t rpm;  //18 RPM

      uint16_t ivolt;  //20 Input voltage in tenths [0.1V]
      uint16_t svolt;  //22 Servo voltage in tenths [0.1V]

      int16_t res2;  //24 0x0000 - Reserved

      uint16_t tps;  //26 Throttle position, 0xFFFF for wrong signal [%]
      uint8_t csp;   //28 Cooler servo position [%]

      uint8_t res3;  //29 0x00 - Reserved

      uint16_t FTL;     //30 Fuel tank level [0.1L]
      uint16_t FP;      //32 Fuel pressure [0.1B]
      uint16_t FPREGP;  //34 Fuel pressure regulator output [%]
      uint16_t CFCHPL;  //36 Current fuel consumption in hundreths [0.01 l/h]

      uint8_t LS;  //38 Lambda style, 0 – volt lambda, 1 – AFR lambda

      uint8_t res4;  //39 0x00 - Reserved

      int16_t CLS;  //40 Current lambda setting, -1 = OFF, tens of mV for analog => 70 = 700 mV, tenths of AFR for digital => 80 = 8.0 AFR
      uint16_t LV;  //42 Lambda value, tenths of mV for analog => 70 = 700 mV, tenths of AFR for digital => 80 = 8.0 AFR
      int16_t CLC;  //44 Current lambda correction [0.1%]

      uint16_t ilus;  //46 Injection length in us

      uint8_t sum;  //48 Sum of bytes 0 to 47
      uint8_t eop;  //49 0x0D - End of packet
    } data;
        uint8_t pkt[50];
    };
    size_t ind;
    uint32_t last_recv;  // ms

    uint16_t pkt_nbytes;
    uint32_t last_request_ms;
    uint32_t last_recv_ms;
};

#endif  // AP_EFI_SERIAL_FH_ENABLED
