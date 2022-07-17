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
 
#include <AP_HAL/AP_HAL.h>
#include "AP_EFI_Serial_Lutan.h"
#include <AP_HAL/utility/sparse-endian.h>

#if HAL_EFI_ENABLED

#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// RPM Threshold for fuel consumption estimator
#define RPM_THRESHOLD                100

extern const AP_HAL::HAL &hal;

AP_EFI_Serial_Lutan::AP_EFI_Serial_Lutan(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


void AP_EFI_Serial_Lutan::update()
{
    if (port == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (pkt_nbytes > 0 && now - last_recv_ms > 100) {
        pkt_nbytes = 0;
    }
    const uint32_t n = MIN(sizeof(pkt), port->available());
    if (n > 0) {
        last_recv_ms = now;
    }
    if (n + pkt_nbytes > sizeof(pkt)) {
        pkt_nbytes = 0;
    }
    ssize_t nread = port->read(&pkt[pkt_nbytes], n);
    if (nread < 0) {
        nread = 0;
    }
    pkt_nbytes += nread;
    if (pkt_nbytes > 2) {
        const uint16_t length = be16toh(data.length);
        if (length+6 == pkt_nbytes) {
            // got a pkt of right length
            const uint32_t crc = be32toh_ptr(&pkt[length+2]);
            const uint32_t crc2 = ~crc_crc32(~0U, &pkt[2], length);
            if (crc == crc2) {
                // valid data
                internal_state.spark_dwell_time_ms = int16_t(be16toh(data.dwell))*0.1;
                internal_state.cylinder_status[0].injection_time_ms = be16toh(data.pulseWidth1)*0.00666;
                internal_state.engine_speed_rpm = be16toh(data.rpm);
                internal_state.atmospheric_pressure_kpa = int16_t(be16toh(data.barometer))*0.1;
                internal_state.intake_manifold_pressure_kpa = int16_t(be16toh(data.map))*0.1;
                internal_state.intake_manifold_temperature = degF_to_Kelvin(int16_t(be16toh(data.mat))*0.1);
                internal_state.coolant_temperature = degF_to_Kelvin(int16_t(be16toh(data.coolant))*0.1);
                // CHT is in coolant field
                internal_state.cylinder_status[0].cylinder_head_temperature = internal_state.coolant_temperature;
                internal_state.throttle_position_percent = int16_t(be16toh(data.tps))*0.1;

                // integrate fuel consumption
                if (internal_state.engine_speed_rpm > RPM_THRESHOLD) {
                    const float duty_cycle = (internal_state.cylinder_status[0].injection_time_ms * internal_state.engine_speed_rpm)/600.0f;
                    internal_state.fuel_consumption_rate_cm3pm = duty_cycle*get_coef1() - get_coef2();
                    internal_state.estimated_consumed_fuel_volume_cm3 += internal_state.fuel_consumption_rate_cm3pm * (now - internal_state.last_updated_ms)/60000.0f;
                } else {
                    internal_state.fuel_consumption_rate_cm3pm = 0;
                }
                internal_state.last_updated_ms = now;
                copy_to_frontend();
            }
            pkt_nbytes = 0;
        }
    }
    if (now - last_request_ms > 200) {
        last_request_ms = now;
        port->discard_input();
        send_request();
    }
}

void AP_EFI_Serial_Lutan::send_request(void)
{
    static const uint8_t d[] = { 0, 1, 0x41 };
    const uint32_t crc = ~crc_crc32(~0U, &d[2], sizeof(d)-2);
    const uint32_t crc2 = htobe32(crc);
    port->write(d, sizeof(d));
    port->write((const uint8_t *)&crc2, sizeof(crc2));
}

#endif // HAL_EFI_ENABLED
