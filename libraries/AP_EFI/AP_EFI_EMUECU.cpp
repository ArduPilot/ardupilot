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
#include "AP_EFI_EMUECU.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ICEngine/AP_ICEngine.h>
#include <unistd.h>
#include <stdio.h>

#if EFI_ENABLED
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

AP_EFI_EMUECU::AP_EFI_EMUECU(AP_EFI &_frontend):
    AP_EFI_Backend(_frontend)
{
    port = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_EFI, 0);
}


void AP_EFI_EMUECU::update()
{
    if (!port) {
        return;
    }

    send_throttle();

    uint32_t n = port->available();
    n = MIN(n, 1200U);
    if (n == 0) {
        return;
    }

    while (n--) {
        uint8_t b = port->read();
        linebuf[line_len++] = b;
        if (line_len == sizeof(linebuf)-1) {
            // overflow, reset
            line_len = 0;
            break;
        }
        if (b == '\n') {
            linebuf[line_len-1] = 0;
            // got a full line
            process_line();
            line_len = 0;
        }
    }
}


// send throttle demand
void AP_EFI_EMUECU::send_throttle(void)
{
    if (port->txspace() < 20) {
        return;
    }
    uint16_t throttle = 0;
    AP_HAL::Util::safety_state safety_state = hal.util->safety_switch_state();
    if (hal.util->get_soft_armed() && safety_state != AP_HAL::Util::SAFETY_DISARMED) {
        AP_ICEngine *ice = AP::ice();
        if (!ice || ice->get_state() > AP_ICEngine::ICE_OFF) {
            throttle = 1000 + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 10;
        }
    }

    port->printf("set thr_over %u\r\n", throttle);
}

/*
  process one line of JSON
 */
void AP_EFI_EMUECU::process_line(void)
{
    // special handling of var show
    if (strchr(linebuf, '=') && line_len < 40) {
        gcs().send_text(MAV_SEVERITY_INFO, "EMU: %s", linebuf);
        return;
    }

    // special handling of log msg
    if (strncmp(linebuf, "{\"log\":{\"msg\":\"", 15) == 0 && line_len > 18) {
        const char *msg = &linebuf[15];
        linebuf[line_len-5] = 0;
        if (strlen(msg) < sizeof(last_log_msg)-1 && strcmp(last_log_msg, msg)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "EMU: %s", msg);
            strcpy(last_log_msg, msg);
        }
        return;
    }

    // special handling of var msg
    if (strncmp(linebuf, "{\"var\":{\"", 9) == 0) {
        char *ptr = nullptr;
        const char *varname = strtok_r(&linebuf[9], " :\"{}", &ptr);
        const char *var = strtok_r(nullptr, " :\"{}", &ptr);
        if (varname && var) {
            gcs().send_text(MAV_SEVERITY_WARNING, "EMU: %s=%s", varname, var);
        }
        return;
    }
    
    for (uint16_t i=0; i<ARRAY_SIZE(keytable); i++) {
        const struct keytable &key = keytable[i];

        // find key
        const char *p = strstr(linebuf, key.key);
        if (!p) {
            continue;
        }

        p += strlen(key.key);
        if (*p++ != '"') {
            continue;
        }
        if (*p++ != ':') {
            continue;
        }

        switch (key.type) {
        case DataType::FLOAT:
            *((float *)key.ptr) = strtof(p, nullptr);
            break;

        case DataType::UINT8:
            *((uint8_t *)key.ptr) = strtoul(p, nullptr, 10);
            break;

        case DataType::UINT16:
            *((uint16_t *)key.ptr) = strtoul(p, nullptr, 10);
            break;

        case DataType::INT16:
            *((int16_t *)key.ptr) = strtol(p, nullptr, 10);
            break;

        case DataType::UINT32:
            *((uint32_t *)key.ptr) = strtoul(p, nullptr, 10);
            break;
        }
    }

    internal_state.engine_speed_rpm = status.rpm;
    internal_state.intake_manifold_temperature = status.iat*0.01 + C_TO_KELVIN;
    internal_state.cylinder_status[0].cylinder_head_temperature = status.cht*0.01 + C_TO_KELVIN;
    internal_state.cylinder_status[0].exhaust_gas_temperature = status.egt*0.01 + C_TO_KELVIN;
    internal_state.throttle_position_percent = status.throttle_out;
    internal_state.pt_c = status.pt_c;
    internal_state.atmospheric_pressure_kpa = status.baro * 0.001;
    internal_state.cylinder_status[0].injection_time_ms = status.inj_ticks * 16 * 0.001;
    copy_to_frontend();
}

#endif // EFI_ENABLED
