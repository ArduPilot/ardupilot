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
  simulate EMU EFI system
*/

#include "SIM_Aircraft.h"
#include <SITL/SITL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <stdio.h>

using namespace SITL;

// printf to socket for easy JSON output
void EFI_EMUECU::efi_printf(const char *fmt, ...)
{
    va_list ap;
    char *buf = nullptr;
    va_start(ap, fmt);
    uint32_t n = vasprintf(&buf, fmt, ap);
    va_end(ap);
    if (!buf) {
        return;
    }
    sock.send(buf, n);
    free(buf);
}

void EFI_EMUECU::config_dump()
{
  efi_printf("{\"config\":{");

  efi_printf("\"version\":%u,", (uint16_t)config.version);
  efi_printf("\"thr_min\":%u,", config.thr_min);
  efi_printf("\"thr_start\":%u,", config.thr_start);
  efi_printf("\"thr_max\":%u,", config.thr_max);
  efi_printf("\"pwm0_min\":%u,", config.pwm0_min);
  efi_printf("\"pwm0_max\":%u,", config.pwm0_max);
  efi_printf("\"pwm1_min\":%u,", config.pwm1_min);
  efi_printf("\"pwm1_max\":%u,", config.pwm1_max);
  efi_printf("\"auto_start\":%u,", (uint16_t)config.auto_start);
  efi_printf("\"rpm_limit\":%u,", config.rpm_limit);

  efi_printf("\"capacity\":%u,", (uint16_t)config.capacity);
  efi_printf("\"inj_open\":%u,", config.inj_open);
  efi_printf("\"inj_close\":%u,", config.inj_close);
  efi_printf("\"inj_flow\":%u,", config.inj_flow);

  efi_printf("\"idle_rpm\":%u,", config.idle_rpm);
  efi_printf("\"dwell_time_ms\":%u,", config.dwell_time_ms);
  efi_printf("\"start_time_ms\":%u,", config.start_time_ms);

  efi_printf("}}\n");
}

void EFI_EMUECU::update()
{
    auto sitl = AP::sitl();
    if (!sitl || sitl->efi_type != SITL::EFI_TYPE_EMU) {
        return;
    }
    if (!connected) {
        connected = sock.connect("127.0.0.1", 5763);
    }
    if (!connected) {
        return;
    }
    float rpm = sitl->state.rpm[0];

    status.rpm = rpm;
    uint32_t now = AP_HAL::millis();
    if (now - last_send_ms < 1000) {
        return;
    }
    if (last_send_ms == 0) {
        config_dump();
    }
    last_send_ms = now;

    efi_printf("{\"status\":{\"thr_in\":%d,\"throttle_in\":%d,\"throttle_out\":%d,\"rpm\":%u,\"cht\":%d,\"iat\":%d}}\n",
           status.thr_in, (int)(100*status.throttle_in), (int)(100*status.throttle_out), status.rpm,
           status.cht, status.iat);
    efi_printf("{\"status\":{\"baro\":%lu,\"ecut\":%d,\"humidity\":%u,\"egt\":%lu}}\n",
           status.baro, status.ecut, status.humidity, status.egt);
    efi_printf("{\"status\":{\"pt_c\":%f,\"starts\":%u}}\n",
           status.pt_c, status.starts);
    efi_printf("{\"status\":{\"pwm0_out\":%d,\"pwm1_out\":%d,\"inj_ticks\":%u}}\n",
           status.pwm0_out, status.pwm1_out, 0);
    
}
