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

void AP_EFI_Serial_FH::update()
{
    const uint32_t now = AP_HAL::millis();
    boza++;
    if (process(now)) {
        boza=0x66;
            internal_state.spark_dwell_time_ms = 1234 * 0.1;
            internal_state.cylinder_status.injection_time_ms = data.ilus;
            internal_state.engine_speed_rpm = data.rpm;
            internal_state.atmospheric_pressure_kpa = 2345 * 0.1;
            internal_state.intake_manifold_pressure_kpa = 3456 * 0.1;
            internal_state.intake_manifold_temperature = degF_to_Kelvin(data.tempOut) * 0.1;
            internal_state.coolant_temperature = degF_to_Kelvin(111) * 0.1;
            // CHT is in coolant field
            internal_state.cylinder_status.cylinder_head_temperature = data.temp[1];
            internal_state.throttle_position_percent = data.tps * 0.1;
            // integrate fuel consumption
            if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
                const float duty_cycle = (internal_state.cylinder_status.injection_time_ms * internal_state.engine_speed_rpm) / 600.0f;
                internal_state.fuel_consumption_rate_cm3pm = duty_cycle * get_coef1() - get_coef2();
                internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (now - internal_state.last_updated_ms) / 60000.0f;
            } else {
                internal_state.fuel_consumption_rate_cm3pm = 0;
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
    static uint8_t d[] = { 0, 'A', 'L', 'F', '!' };
    d[0]=boza;
 //   const uint32_t crc = ~crc_crc32(~0U, &d[2], sizeof(d)-2);
//    const uint32_t crc2 = htobe32(crc);
    port->write(d, sizeof(d));
//    port->write((const uint8_t *)&crc2, sizeof(crc2));
}

#endif  // AP_EFI_SERIAL_FH_ENABLED
