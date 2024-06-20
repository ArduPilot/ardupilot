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

#include "AP_EFI_config.h"

#if AP_EFI_SERIAL_FH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "AP_EFI_Serial_FH.h"

#include <stdio.h>

// RPM Threshold for fuel consumption estimator
#define RPM_THRESHOLD                100

extern const AP_HAL::HAL &hal;


AP_EFI_Serial_FH::AP_EFI_Serial_FH(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);

    // Indicate that temperature and fuel pressure are supported
    internal_state.fuel_pressure_status = Fuel_Pressure_Status::OK;
    internal_state.temperature_status = Temperature_Status::OK;

    // FlyHenry ECU reports EGT 
    internal_state.cylinder_status.exhaust_gas_temperature = 0;
    internal_state.cylinder_status.exhaust_gas_temperature2 = 0;

}


  bool AP_EFI_Serial_FH::process(uint32_t const now) {
    if (port != nullptr) {
      if ((ind && (now - last_recv > 300)) || (ind >= sizeof(pkt))) ind = 0;
      size_t n = MIN(sizeof(pkt) - ind, port->available());
      if (n) {
        last_recv = now;
        while (n--) {
          uint8_t i = ind;
          pkt[ind++] = port->read();
          const uint8_t hdr[] = { 0xA5, 0x5A, 1, 0 };
          if (i < sizeof(hdr) && pkt[i] != hdr[i]) ind = 0;
          else if (sizeof(pkt) == ind) return (0xD == data.eop && data.sum == sum8(pkt, 48));
        }
      }
    }
    return false;
  }

static unsigned char boza = 0;
#include "../AP_ICEngine/AP_ICEngine.h"

void AP_EFI_Serial_FH::update()
{
    const uint32_t now = AP_HAL::millis();
    boza++;
    if (process(now)) {
        boza=0x66;
        internal_state.spark_dwell_time_ms = 123.45f;        
        // RPM and Engine State
        internal_state.engine_speed_rpm = data.rpm;
        if (internal_state.engine_speed_rpm > 0) {
          internal_state.engine_state = Engine_State::RUNNING;
        } else {
          internal_state.engine_state = Engine_State::STOPPED;
        }
        // EFI ECU power [volts]
        internal_state.ignition_voltage = tenth(data.ivolt);
        internal_state.atmospheric_pressure_kpa = tenth(data.ivolt);
        //0=-273.15 273.15=0
        internal_state.intake_manifold_temperature = degC_to_Kelvin(data.tempOut);
        internal_state.intake_manifold_pressure_kpa = tenth(data.FP);
        
        auto &c = internal_state.cylinder_status;
        // CHT temp 1/2
        c.cylinder_head_temperature  = degC_to_Kelvin(MAX(data.temp[0], data.temp[1]));
        c.cylinder_head_temperature2 = degC_to_Kelvin(MIN(data.temp[0], data.temp[1]));
        // EGT temp 3/4
        c.exhaust_gas_temperature = degC_to_Kelvin(MAX(data.temp[2], data.temp[3]));
        c.exhaust_gas_temperature2 = degC_to_Kelvin(MIN(data.temp[2], data.temp[3]));
        // PMU/GEN temp
        internal_state.coolant_temperature = degC_to_Kelvin(data.temp[4]);
        internal_state.oil_temperature = degC_to_Kelvin(data.temp[5]);
        // Injection timing
        c.injection_time_ms = thousnd(data.ilus);            
        // Lambda
        c.lambda_coefficient = tenth(data.LV);
        // Throttle Position [%]
        internal_state.throttle_position_percent = data.tps;

        internal_state.fuel_pressure = tenth(data.FP); // Fuel_Pressure_Status::OK;
        // Fuel consumption
        internal_state.fuel_consumption_rate_cm3pm = convertFuelConsumption((float)data.CFCHPL/1000.f);
        // Integrate fuel consumption
        if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
            //const float duty_cycle = (internal_state.cylinder_status.injection_time_ms * internal_state.engine_speed_rpm) / 600.0f;
            //internal_state.fuel_consumption_rate_cm3pm = duty_cycle * get_coef1() - get_coef2();
            internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (now - internal_state.last_updated_ms) / 60000.0f;
        } else {
            //internal_state.fuel_consumption_rate_cm3pm = 0;
        }
        internal_state.last_updated_ms = now;
        copy_to_frontend();
    }
    if (now - last_request_ms > 200) {
        last_request_ms = now;
        port->discard_input();
        send_request();
    }
}

void AP_EFI_Serial_FH::send_request(void)
{
    static uint8_t d[] = { 0, 0, 'A', 'L', 'F' };

    uint8_t kur=0;
    switch (AP::ice()->state) {

      case AP_ICEngine::ICE_DISABLED: kur = 0xFF; break;
      case AP_ICEngine::ICE_OFF: kur = 0; break;
      case AP_ICEngine::ICE_START_HEIGHT_DELAY:
      case AP_ICEngine::ICE_START_DELAY: kur = 1; break;
      case AP_ICEngine::ICE_STARTING: kur = 2; break;    
      case AP_ICEngine::ICE_RUNNING: kur = 3; break;
      default:break;
    }

    d[0] = boza;
    d[1] = kur;
 //   const uint32_t crc = ~crc_crc32(~0U, &d[2], sizeof(d)-2);
//    const uint32_t crc2 = htobe32(crc);
    port->write(d, sizeof(d));
//    port->write((const uint8_t *)&crc2, sizeof(crc2));
}

#endif  // AP_EFI_SERIAL_FH_ENABLED
